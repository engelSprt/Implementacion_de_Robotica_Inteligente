#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class BlackDetector:
    def __init__(self):
        # Coordenadas del area de interes (ROI)
        self.roi_x = 0
        self.roi_y = 500
        self.roi_width = 1000
        self.roi_height = 400

        # Resolucion de la imagen fija
        self.image_width = 3280
        self.image_height = 2464

        self.kernel = np.ones((5, 5), np.uint8)

        # Crear el objeto CvBridge
        self.bridge = CvBridge()

        # Suscribirse al topico de la fuente de video en bruto
        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 

        # Crear el publicador para el set_point
        self.actual_pos_pub = rospy.Publisher('/actual_pos', Point, queue_size=10)
        self.error_pub = rospy.Publisher('/line_error', Float64, queue_size =10)
        self.mask_pub = rospy.Publisher('/mask_cut',Image, queue_size=10)
        self.cut_pub = rospy.Publisher('/frame_cut',Image, queue_size=10)

    def image_callback(self, msg):
        # Convertir la imagen de ROS a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Recorte
        cut = frame[self.roi_y:self.roi_y + self.roi_height, self.roi_x:self.roi_x + self.roi_width]

        blurred_frame = cv2.GaussianBlur(cut, (5, 5), 0)

        gray = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2GRAY)

        # Umbral
        _, threshold = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

        clean = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, self.kernel)
        clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, self.kernel)
        clean = cv2.dilate(clean, self.kernel, iterations=1)

        # Encontrar los contornos de las regiones negras
        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Punto medio

        desired_x=500
        desired_y=200
        
        # Dibujar contornos
        cv2.drawContours(clean, contours, -1, (0, 255, 0), 2)
        cv2.drawContours(cut, contours, -1, (0, 255, 0), 2)

        # Calcular el set point en funcion de la posicion del contorno mas grande
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                set_point = (cx,cy)
                rospy.loginfo("Actual point: %s", set_point)
                # Publicar el valor de set_point en el topico /set_point
                set_point_msg = Point()
                set_point_msg.x = set_point[0]
                set_point_msg.y = set_point[1]
                self.actual_pos_pub.publish(set_point_msg)


        # Dibujar puntos 
        cv2.circle(clean, (desired_x,desired_y), 7, (0,0,255),-1)
        cv2.circle(cut, (desired_x,desired_y), 7, (0,0,255),-1)
        cv2.circle(clean, (cx, desired_y), 7, (0,255,0),-1)
        cv2.circle(cut, (cx, desired_y), 7, (0,255,0),-1)

        # Imprimir error
        cv2.putText(clean, str(desired_x - cx), (cx+20, desired_y), cv2. FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2,cv2.LINE_AA)
        cv2.putText(cut, str(desired_x - cx), (cx+20, desired_y), cv2. FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2,cv2.LINE_AA)
        self.error_pub.publish(desired_x - cx)

        #se convierten las mascaras de tipo imagen de cv2 a mensajes para publicar el topico
        mask_cut = self.bridge.cv2_to_imgmsg(clean)
        frame_cut = self.bridge.cv2_to_imgmsg(cut, encoding='bgr8')
        
        self.mask_pub.publish(mask_cut)
        self.cut_pub.publish(frame_cut)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    # Inicializar el nodo ROS
    rospy.init_node('black_detector')
    black_detector = BlackDetector()
    black_detector.run()
    cv2.destroyAllWindows()