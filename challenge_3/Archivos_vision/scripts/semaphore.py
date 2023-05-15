
#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class TrafficLightDetector:
    def __init__(self):

        #Se definen los rangos de color en HSV
        self.lower_red = np.array([38, 86, 0])
        self.upper_red = np.array([121, 255, 255])

        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])

        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([90, 255, 255])

        self.kernel = np.ones((5, 5), np.uint8)

        self.bridge = cv_bridge.CvBridge()

        #Se definen suscriptores y publicadores
        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 
        self.traffic_light_pub = rospy.Publisher('/traffic_light', String, queue_size=10)

        self.mask_red_pub = rospy.Publisher('/red_mask',Image, queue_size=10)
        self.mask_yellow_pub = rospy.Publisher('/yellow_mask',Image, queue_size=10)
        self.mask_green_pub = rospy.Publisher('/green_mask',Image, queue_size=10)        

    def image_callback(self, data):
        #se convierte el frame de tipo mensaje a imagen de cv2 para procesarla
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #Se aplica un filtro Gaussiano para el ruido
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
        #Se convierte el fotograma de BGR a HSV 
        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

        #Se define la mascara para el rango de color del rojo
        mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)
        #Se aplican operaciones morfolgicas a la mascara para eliminar ruido
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, self.kernel)
        contoursRed, hierarchy = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        #Con los contornos se calcula el area del objeto del color detectado
        for contour in contoursRed:
            area_Red = cv2.contourArea(contour)
            #Si el area es suficientemente grande y el color esta dentro del rango se publica el mensaje
            if area_Red >6000 and cv2.countNonZero(mask_red) > 100:
                self.traffic_light_pub.publish("Stop")

        #Se define la mascara para el rango de color del amarillo
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        #Se aplican operaciones morfolgicas a la mascara para eliminar ruido
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, self.kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, self.kernel)
        contoursYellow, hierarchy = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        #Con los contornos se calcula el area del objeto del color detect
        for contour in contoursYellow:
            area_yellow = cv2.contourArea(contour)
            #Si el area es suficientemente grande y el color esta dentro del rango se publica el mensaje
            if area_yellow >6000 and cv2.countNonZero(mask_yellow) > 100:
                self.traffic_light_pub.publish("Slow_down")

        #Se define la mascara para el rango de color del verde
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        #Se aplican operaciones morfolgicas a la mascara para eliminar ruido
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, self.kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, self.kernel)
        contoursGreen, hierarchy = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        #Con los contornos se calcula el area del objeto del color detect
        for contour in contoursGreen:
            area_green = cv2.contourArea(contour)
            #Si el area es suficientemente grande y el color esta dentro del rango se publica el mensaje
            if area_green >6000 and cv2.countNonZero(mask_green) > 100:
                self.traffic_light_pub.publish("Go")

        #se convierten las mascaras de tipo imagen de cv2 a mensajes para publicar el topico
        maskR = self.bridge.cv2_to_imgmsg(mask_red)
        maskY = self.bridge.cv2_to_imgmsg(mask_yellow)
        maskG = self.bridge.cv2_to_imgmsg(mask_green)
        
        self.mask_red_pub.publish(maskR)
        self.mask_yellow_pub.publish(maskY)
        self.mask_green_pub.publish(maskG)

        #cv2.imshow("Vision", frame)
        #cv2.imshow("Mask red", mask_red)
        #cv2.imshow("Mask yellow", mask_yellow)
        #cv2.imshow("Mask green", mask_green)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_detector')
    traffic_light_detector = TrafficLightDetector()
    traffic_light_detector.run()
    cv2.destroyAllWindows()
    