#!/usr/bin/env python3

import rclpy
import os, time, tempfile
import cv2
import numpy as np
from typing import NamedTuple
from threading import Thread, Timer

from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose as PoseMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

# Parametros

IGNORE_VALUE = 100000.0
DISTSETPOINT = 500.0
ERROR_UPPER_BOUNDRY = 5
ERROR_LOWER_BOUNDRY = 5
LINEAR_SPEED = 0.2

class SeguidorDeObjeto(Node):
    def __init__(self, 
                 id:str,
                #  rgb_topic      : str = "/camera/rgb/image_color",
                 depth_topic    : str = "/depth/image_raw",
                 sqr_pos_topic  : str = "blue_square_position",
                 state_topic  : str = "rot_state",
                 x_obj_topic    : str = "setpoint_rotational",
                 distance_topic : str = "dist_state",
                 ):
        super().__init__(id)

        self.subscription_depth = self.create_subscription(Image, depth_topic, self.depth_img_cb, 1)
        self.sqr_pos_subscriber = self.create_subscription(Vector3, sqr_pos_topic, self.follow_the_blue_boxy_carrot, 1)
        self.state_publisher  = self.create_publisher(Vector3, state_topic, 1)
        self.forward_publisher  = self.create_publisher(Float64, distance_topic, 1)
        self.actuation_pub = self.create_publisher( Twist, '/commands/velocity', 1 )
        
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.image_width = None

    def depth_img_cb(self, depth_img: Image):
        if depth_img != None:
            self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(depth_img)
        else:
            self.get_logger().info( f"No depth image received" )

    def follow_the_blue_boxy_carrot(self, position: Vector3):
        '''
        Si se quiere girar y avanzar, se tendria que enviar la diferencia en x y la distancia en un mensaje
        al controlador. Ahi, se deberia crear el mensaje para rotar y dependiendo de la distancia al
        objeto, tambien se deberia editar la velocdad linear.
        Hipotesis: si el objeto entra en la zona considerada alineada, se va a dejar de agregar error.
        Por esto, si el objeto sale de la zona por el mismo lado por donde entro, el robot tendra
        un undershoot por el error pasado.
        '''

        def square_depth(cv_depth_image, x: float, y: float) -> int:
            distance = float(cv_depth_image[int(y), int(x)])
            return distance

        if position != None:  
            self.relative_x = position.x
            self.get_logger().info( f"square_rel_x_position: {self.relative_x}" ) 
            self.y = position.y
            self.image_width = position.z
            center_of_mass_square_x = (self.image_width/2) - self.relative_x    # Podria ser simplemente el centro de la imagen (?)
            square_depth = square_depth(self.current_cv_depth_image, center_of_mass_square_x, self.y)
            self.get_logger().info( f"square_depth type: {type(square_depth)} distance: {square_depth}" )

            # Enviar error de rotacion y distancia al objeto
            msg_pi = Vector3()
            msg_pi.x = self.relative_x
            msg_pi.z = square_depth
            self.state_publisher.publish(msg_pi)


def main(args=None):
    rclpy.init(args=args)
    
    seguidor = SeguidorDeObjeto("blue_square_detector")
    rclpy.spin(seguidor)

    seguidor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

