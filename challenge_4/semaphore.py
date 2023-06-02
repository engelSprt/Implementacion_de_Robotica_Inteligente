
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
        self.lower_red = np.array([70, 192, 130])
        self.upper_red = np.array([180, 255, 255])

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
            if area_Red >5000 and cv2.countNonZero(mask_red) > 2000:
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
            if area_yellow >5000 and cv2.countNonZero(mask_yellow) > 2000:
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
            if area_green >5000 and cv2.countNonZero(mask_green) > 2000:
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


'''
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
        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 
        self.image_pub = rospy.Publisher("masked_image", Image, queue_size=1)
        
        cv2.namedWindow("Parameters")
        cv2.createTrackbar("L-H", "Parameters", 0,180, self.nothing)
        cv2.createTrackbar("L-S", "Parameters", 68, 255, self.nothing)
        cv2.createTrackbar("L-V", "Parameters", 154, 255, self.nothing)
        cv2.createTrackbar("U-H", "Parameters", 180, 180, self.nothing)
        cv2.createTrackbar("U-S", "Parameters", 255, 255, self.nothing)
        cv2.createTrackbar("U-V", "Parameters", 243, 255, self.nothing)
        
        #self.cap = cv2.VideoCapture(0)

    def nothing(self, x):
        pass

    def run(self):
        while not rospy.is_shutdown():
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
                if area >1000:
                    cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)

            #cv2.imshow("Color detection", frame)
            #cv2.imshow("Mask", mask)

            # Publicar la imagen mascarada en el tópico "masked_image"
            msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            self.image_pub.publish(msg)


if __name__ == '__main__':
    try:
        cd = ColorDetector()
        cd.run()
    except rospy.ROSInterruptException:
        pass

'''