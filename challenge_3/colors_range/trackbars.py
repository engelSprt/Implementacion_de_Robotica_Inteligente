###sin ros
import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow("Parameters")
cv2.createTrackbar("L-H", "Parameters", 0,180, nothing)
cv2.createTrackbar("L-S", "Parameters", 68, 255, nothing)
cv2.createTrackbar("L-V", "Parameters", 154, 255, nothing)
cv2.createTrackbar("U-H", "Parameters", 180, 180, nothing)
cv2.createTrackbar("U-S", "Parameters", 255, 255, nothing)
cv2.createTrackbar("U-V", "Parameters", 243, 255, nothing)


# Capturar video en vivo desde la cmara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L-H", "Parameters")
    l_s = cv2.getTrackbarPos("L-S", "Parameters")
    l_v = cv2.getTrackbarPos("L-V", "Parameters")
    u_h = cv2.getTrackbarPos("U-H", "Parameters")
    u_s = cv2.getTrackbarPos("U-S", "Parameters")
    u_v = cv2.getTrackbarPos("U-V", "Parameters")
    
    
    lower_color = np.array([l_h, l_s, l_v])
    upper_color = np.array([u_h, u_s, u_v])
    
    # Aplicar una mscara al fotograma para aislar los pixeles
    mask = cv2.inRange(hsv, lower_color, upper_color)
    masked = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Aplicar un filtro de borde para detectar los bordes de los objetos
    edges = cv2.Canny(masked, 100, 200)
    
    # Encontrar los contornos de los objetos en la imagen y dibujarlos
    _,contours,_= cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        
        if area >1000:
            cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)
    
    # Mostrar el fotograma con los objetos detectados
    cv2.imshow("Color detection", frame)
    cv2.imshow("Mask", mask)
    
    # Salir del bucle si se presiona la tecla 'esc'
    key = cv2.waitKey(1)
    if key == 27:
        break

# Liberar la cmara y cerrar la ventana
cap.release()
cv2.destroyAllWindows()


#### con ros

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