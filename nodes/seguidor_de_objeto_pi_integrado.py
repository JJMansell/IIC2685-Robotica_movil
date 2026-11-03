#!/usr/bin/env python3

import rclpy
import os, time, tempfile
import cv2
import numpy as np
from typing import NamedTuple
from threading import Thread, Timer

from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose as PoseMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

# Parametros

ANGLEKP = 0.5
ANGLEKI = 0.0

# class SpeedCommand(NamedTuple):
#     linear: float
#     angular: float
#     time: float

class SeguidorDeObjeto(Node):
    def __init__(self, 
                 id:str,
                 rgb_topic: str = "/camera/rgb/image_color",
                #  depth_topic: str = "/camera/depth/image_raw",
                 sqr_pos_topic: str = "blue_square_position",
                #  x_state_topic: str = "rot_state",
                #  x_obj_topic: str = "setpoint_rotational"
                 velocity_topic: str = "/commands/velocity",
                 ):
        super().__init__(id)

        self.depth_subscriber = self.create_subscription(Img, depth_topic, self.depth_img_cb, 1)
        self.sqr_pos_subscriber = self.create_subscription(Vector3, sqr_pos_topic, self.follow_the_blue_boxy_carrot, 1)
        # self.x_state_publisher = self.create_publisher(Float64, x_state_topic, 1)
        self.velocity_publisher = self.create_publisher(Float64, velocity_topic, 1)
        self.bridge = CvBridge()

        self.current_cv_depth_image     = None
        self.image_width                = None
        self.state                      = None
        self.prev_state                 = None
        self.angle_setpoint             = 0.0
        self.angle_accumulated_error    = 0.0

        # self.x_obj_publisher = self.create_publisher(Float64, x_obj_topic, 1)
        # while (self.x_obj_publisher.get_subscription_count() == 0 or self.image_width == None) and rclpy.ok():
        #     time.sleep( 0.2 )
        # self.x_obj_publisher.publish(float(int(self.image_width)/2))

    def depth_img_cb(self, depth_img: Image):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cvs2(depth_img)

    def follow_the_blue_boxy_carrot(self, position: Vector3):

        def square_depth(cv_depth_image, x: int, y: int) -> int:
            contour_mask = np.zeros(cv_depth_image.shape[:2], dtype=np.uint8)
            cv2.circle(contour_mask, (x, y), 5, (255, 255, 255), -1)
            # contours, _ = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # blue_square = contours[-1]
            extracted_values = cv2.bitwise_and(cv_depth_image, cv_depth_image, mask=contour_mask)   # Extraemos valores del depth image segund un circulo
            unique_values, counts = np.unique(extracted_values, axis=0, return_counts=True)         # Queremos sacar la moda (si por alguna razon hay vacio, no queremos promediar)
            mode_index = np.argmax(counts)      # Vemos cual se repite mas
            mode_value = unique_values[mode_index]
            return mode_value

        def pi_controller(self, kp, ki, setpoint, state, prev_state, accumulated_error):

            current_time = time.time()
            dt = current_time - self.prev_time
            self.prev_time = current_time
            
            # Proportional
            p_actuation = kp*error

            # Integrative 
            accumulated_error += ( error * dt )
            i_actuation = ki*(accumulated_error)

            # Actuation
            actuation = p_actuation + i_actuation + d_actuation

            return actuation

        self.prev_angle_state = self.angle_state   
        self.angle_state = position.x
        self.y = position.y
        self.image_width = position.z
        self.real_x = self.x + (self.image_width/2)

        self.prev_time = time.time()
        self.square_distance = square_depth(self.cv_depth_image, self.real_x, self.y)

# ===============================

        angle_error = self.angle_setpoint - self.angle_state

        if angle_error < -0.1 or angle_error > 0.1:
            angular_actuation = pi_controller(  self, 
                                                kp= ANGLEKP, 
                                                ANGLEKI, 
                                                self.angle_setpoint, 
                                                self.angle_state, 
                                                self.prev_angle_state, 
                                                self.angle_accumulated_error)

            msg = Twist()
            msg.angular.y = angular_actuation
            self.velocity_publisher.publish( msg )

# ===============================

        # if self.current_cv_depth_image != None:
        #     square_depth = square_depth(self.current_cv_depth_image, x, self.y)
            



            

def main(args=None):
    rclpy.init(args=args)
    
    detector  = DetectorDeObjeto("blue_square_detector")
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

