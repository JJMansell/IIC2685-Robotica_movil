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

# class SpeedCommand(NamedTuple):
#     linear: float
#     angular: float
#     time: float

class SeguidorDeObjeto(Node):
    def __init__(self, 
                 id:str,
                #  rgb_topic      : str = "/camera/rgb/image_color",
                 depth_topic    : str = "/camera/depth/image_raw",
                 sqr_pos_topic  : str = "blue_square_position",
                 x_state_topic  : str = "rot_state",
                 x_obj_topic    : str = "setpoint_rotational",
                 distance_topic : str = "dist_state",
                 ):
        super().__init__(id)

        self.subscription_depth = self.create_subscription(Image, depth_topic, self.depth_img_cb, 1)
        self.sqr_pos_subscriber = self.create_subscription(Vector3, sqr_pos_topic, self.follow_the_blue_boxy_carrot, 1)
        self.x_state_publisher  = self.create_publisher(Float64, x_state_topic, 1)
        self.forward_publisher  = self.create_publisher(Float64, distance_topic, 1)
        
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.image_width = None

        # self.x_obj_publisher = self.create_publisher(Float64, x_obj_topic, 1)
        # while (self.x_obj_publisher.get_subscription_count() == 0 or self.image_width == None) and rclpy.ok():
        #     time.sleep( 0.2 )
        # self.x_obj_publisher.publish(float(int(self.image_width)/2))

    def depth_img_cb(self, depth_img: Image):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(depth_img)

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
            
        self.relative_x = position.x
        self.y = position.y
        self.image_width = position.z
        center_of_mass_square_x = (self.image_width/2) - self.relative_x    # Podria ser simplemente el centro de la imagen (?)

        if not (-0.1 < self.relative_x < 0.1):
            self.x_state_publisher.publish(self.relative_x)

        elif (self.current_cv_depth_image != None):
            self.x_state_publisher.publish(0.0)
            square_depth = square_depth(self.current_cv_depth_image, center_of_mass_square_x, self.y)
            self.forward_publisher.publish()


def main(args=None):
    rclpy.init(args=args)
    
    seguidor = SeguidorDeObjeto("blue_square_detector")
    rclpy.spin(seguidor)

    seguidor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

