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

LOWERBLUE = np.array([100, 100, 100])
UPPERBLUE = np.array([140, 255, 255])
ACTIVATE_DISPLAY = True

# class SpeedCommand(NamedTuple):
#     linear: float
#     angular: float
#     time: float

class DetectorDeObjeto(Node):
    def __init__(self, 
                 id:str,
                 rgb_topic: str = "/image_raw",
                #  depth_topic: str = "/camera/depth/image_raw",
                 sqr_pos_topic: str = "blue_square_position",
                 debug_topic: str = "contour_image"
                 ):
        super().__init__(id)

        self.subscription_rgb = self.create_subscription(Image, rgb_topic, self.rgb_img_cb, 1)
        # self.subscription_depth = self.create_subscription(Img, depth_topic, self.depth_img_cb, 1)
        self.sqr_pos_publisher = self.create_publisher(Vector3, sqr_pos_topic, 1)
        self.contour_img_publisher = self.create_publisher(Image, debug_topic, 1)
        self.bridge = CvBridge()
        self.current_cv_rgb_image = None
        # self.current_cv_depth_image = None

    def rgb_img_cb(self, img_data):

        def extract_x_pos( cv_img ):
            '''
            Se tienen varias lineas de codigo comentadas para comparar opciones
            TO DO
            Revisar color picker
            Enviar como mensaje ros
            '''
            # hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv_img, LOWERBLUE, UPPERBLUE)
            kernel = np.ones((16,16),np.uint8)
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # Limpia ruido
            result = cv2.bitwise_and(cv_img, cv_img, mask=opening)

            # Opcion 1 - Center of mass
            contours, _ = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                big_blue_square = contours[-1]
                m0 = cv2.moments(big_blue_square)
                cx0 = int(m0['m10']/m0['m00'])
                cy0 = int(m0['m01']/m0['m00'])
                cv2.circle(result, (cx0,cy0), 5, (0,0,255), 2)
                print(cx0)

            # Opcion 2 - Bounding box
                x,y,w,h = cv2.boundingRect(big_blue_square)
                cv2.rectangle(result,(x,y),(x+w,y+h),(0,255,0),2)

            # print(len(contours))
            # center_of_square = int((x + w))
            # print(center_of_square)

            # Para testear
                if ACTIVATE_DISPLAY:
                    cv2.imshow('Original Image', cv_img)
                    cv2.imshow('Blue Mask', opening)
                    cv2.imshow('Extracted Object', result)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                # contour_img_msg = Image()
                # contour_img_msg.height = opening.shape[0]
                # contour_img_msg.width = opening.shape[1]
                # contour_img_msg = self.bridge.cv2_to_imgmsg(opening)
                # self.get_logger().info( f"Type opening: {opening}" )
                # self.get_logger().info( f"Type de self.bridge: {contour_img_msg.data}" )
                # self.contour_img_publisher.publish(contour_img_msg)

                return cx0, cy0

            else:
                return (None, None)

        def send_coordinates(x, y, img_width):
            msg = Vector3()
            msg.x = float(x)
            msg.y = float(y)   
            msg.z = float(img_width)
            self.sqr_pos_publisher.publish(msg)
            self.get_logger().info( f"msg sent!" )

        # self.current_cv_rgb_image = self.bridge.imgmsg_to_cv2(img_data)
        # self.current_cv_rgb_image = self.bridge.imgmsg_to_cv2(img_data, 'rgb8')
        image_encoding = img_data.encoding
        # self.get_logger().info( f"image encoding is: {image_encoding}" )
        self.current_cv_rgb_image = self.bridge.imgmsg_to_cv2(img_data, image_encoding)
        if len(self.current_cv_rgb_image) != None or 0:
            if len(self.current_cv_rgb_image.shape[:]) == 3 :
                img_width = self.current_cv_rgb_image.shape[1]
                x_pos, y_pos = extract_x_pos(self.current_cv_rgb_image)
                self.get_logger().info( f"square x coordinate: {x_pos}" )
                self.get_logger().info( f"square y coordinate: {y_pos}" )
                if x_pos != None:
                    x_relative_to_center = float((img_width/2) - x_pos) # Positivo a la izquierda
                    self.get_logger().info( f"x_relative_to_center: {x_relative_to_center}" )
                    send_coordinates(x_relative_to_center, y_pos, img_width)
        else:
            self.get_logger().info( f"No image received" )

    # def depth_img_cb(self, img_data)  
    #     self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(img_data)

def main(args=None):
    rclpy.init(args=args)
    
    detector  = DetectorDeObjeto("blue_square_detector")
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

