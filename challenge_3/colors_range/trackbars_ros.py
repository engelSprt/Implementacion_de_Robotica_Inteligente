#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/mask', Image, queue_size=1)
        cv2.namedWindow("Parameters")
        cv2.createTrackbar("L-H", "Parameters", 0, 180, self.nothing)
        cv2.createTrackbar("L-S", "Parameters", 68, 255, self.nothing)
        cv2.createTrackbar("L-V", "Parameters", 154, 255, self.nothing)
        cv2.createTrackbar("U-H", "Parameters", 180, 180, self.nothing)
        cv2.createTrackbar("U-S", "Parameters", 255, 255, self.nothing)
        cv2.createTrackbar("U-V", "Parameters", 243, 255, self.nothing)

    def nothing(self, x):
        pass

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        l_h = cv2.getTrackbarPos("L-H", "Parameters")
        l_s = cv2.getTrackbarPos("L-S", "Parameters")
        l_v = cv2.getTrackbarPos("L-V", "Parameters")
        u_h = cv2.getTrackbarPos("U-H", "Parameters")
        u_s = cv2.getTrackbarPos("U-S", "Parameters")
        u_v = cv2.getTrackbarPos("U-V", "Parameters")

        lower_color = np.array([l_h, l_s, l_v])
        upper_color = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv, lower_color, upper_color)
        masked = cv2.bitwise_and(frame, frame, mask=mask)
        edges = cv2.Canny(masked, 100, 200)

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)

        # Mostrar el fotograma con los objetos detectados
        cv2.imshow("Color detection", frame)
        cv2.imshow("Mask", mask)

        # Publicar la imagen resultante
        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.image_pub.publish(mask_msg)

        cv2.waitKey(3)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = ColorDetector()
    detector.run()
