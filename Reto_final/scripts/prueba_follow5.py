#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

def obtener_centro_areas_negras(imagen):
    imagen_gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

    _, imagen_binaria = cv2.threshold(imagen_gris, 85, 255, cv2.THRESH_BINARY_INV)

    # Publicar la imagen con los dibujos en el topico "/image_with_detections"
    image_bin_msg = bridge.cv2_to_imgmsg(imagen_binaria, "mono8")
    thresh_pub.publish(image_bin_msg)

    mitad_altura = imagen_binaria.shape[0] // 2
    imagen_borde_superior = imagen_binaria[0:mitad_altura, :]
    imagen_borde_inferior = imagen_binaria[mitad_altura:, :]

    contornos_superior, _ = cv2.findContours(imagen_borde_superior, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centro_superior = None
    if len(contornos_superior) > 0:
        contorno_mas_grande_superior = max(contornos_superior, key=cv2.contourArea)
        momentos_superior = cv2.moments(contorno_mas_grande_superior)
        if momentos_superior["m00"] != 0:
            centro_superior = (int(momentos_superior["m10"] / momentos_superior["m00"]),
                               int(momentos_superior["m01"] / momentos_superior["m00"]))

    contornos_inferior, _ = cv2.findContours(imagen_borde_inferior, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centro_inferior = None
    if len(contornos_inferior) > 0:
        contorno_mas_grande_inferior = max(contornos_inferior, key=cv2.contourArea)
        momentos_inferior = cv2.moments(contorno_mas_grande_inferior)
        if momentos_inferior["m00"] != 0:
            centro_inferior = (int(momentos_inferior["m10"] / momentos_inferior["m00"]),
                               int(momentos_inferior["m01"] / momentos_inferior["m00"]))

    return centro_superior, centro_inferior

def process_frame(frame):
    global bridge, centroides_pub, angle_pub, image_pub, nose_pub

    roi_top = 340
    roi_bottom = 640

    roi_width = 650
    frame_height, frame_width, _ = frame.shape
    roi_left = (frame_width - roi_width) // 2
    roi_right = roi_left + roi_width

    roi = frame[roi_top:roi_bottom, roi_left:roi_right]

    centro_superior, centro_inferior = obtener_centro_areas_negras(roi)

    if centro_superior is not None:
        centro_superior = (centro_superior[0] + roi_left, centro_superior[1] + roi_top)
        cv2.circle(frame, centro_superior, 5, (0, 255, 0), -1)

    if centro_inferior is not None:
        mitad_altura = roi_bottom - roi_top
        centro_inferior = (centro_inferior[0] + roi_left, centro_inferior[1] + roi_top + mitad_altura - 30)
        cv2.circle(frame, centro_inferior, 5, (0, 255, 0), -1)

        # Calcular el angulo
        dx = centro_superior[0] - centro_inferior[0]
        dy = centro_superior[1] - centro_inferior[1]

        if abs(dx) < 1 and abs(dy) < 1:
            angulo_grados = 0
        else:
            angulo_radianes = math.atan2(dy, dx)
            angulo_grados = math.degrees(angulo_radianes)

            # Ajustar el angulo para que este en el rango de 0 a 360 grados
            angulo_grados %= 360
            angulo_grados = angulo_grados - 270

        cv2.putText(frame, "Angulo: {:.2f} grados".format(angulo_grados), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Dibujar linea que une los centroides
        cv2.line(frame, centro_inferior, centro_superior, (204, 204, 255), 2)

        # Publicar los centroides en el topico "/centroides"
        centroides_msg = PointStamped()
        centroides_msg.header.stamp = rospy.Time.now()
        centroides_msg.header.frame_id = "camera_frame"
        centroides_msg.point.x = centro_inferior[0]
        centroides_msg.point.y = centro_inferior[1]

        centroides_pub.publish(centroides_msg)

        # Publicar el angulo en el topico "/ang_err"
        nose_pub.publish(6.1)
        if angulo_grados <= 35 and angulo_grados >= -35:
            if angulo_grados <= 8 and angulo_grados >= -8:
                angle_pub.publish(angulo_grados)*(-1)
            else:
                angle_pub.publish(angulo_grados)
        angle_pub.publish(angulo_grados)
        nose_pub.publish(5.1)

    # Publicar la imagen con los dibujos en el topico "/image_with_detections"
    image_with_detections_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    image_pub.publish(image_with_detections_msg)

def image_callback(msg):
    global bridge

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    process_frame(frame)

def main():
    global bridge, centroides_pub, angle_pub, image_pub, nose_pub, thresh_pub

    rospy.init_node("image_processing_node")
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/video_source/raw", Image, image_callback)
    centroides_pub = rospy.Publisher("/centroides", PointStamped, queue_size=1)
    angle_pub = rospy.Publisher("/ang_err", Float32, queue_size=1)
    nose_pub = rospy.Publisher("/nose", Float32, queue_size=1)
    image_pub = rospy.Publisher("/image_with_detections", Image, queue_size=1)
    thresh_pub = rospy.Publisher("/thresh", Image, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()


'''
#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

def obtener_centro_areas_negras(imagen):
    imagen_gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

    _, imagen_binaria = cv2.threshold(imagen_gris, 100, 255, cv2.THRESH_BINARY_INV)

    # Publicar la imagen con los dibujos en el topico "/image_with_detections"
    image_bin_msg = bridge.cv2_to_imgmsg(imagen_binaria, "mono8")
    thresh_pub.publish(image_bin_msg)

    mitad_altura = imagen_binaria.shape[0] // 2
    imagen_borde_superior = imagen_binaria[0:mitad_altura, :]
    imagen_borde_inferior = imagen_binaria[mitad_altura:, :]

    contornos_superior, _ = cv2.findContours(imagen_borde_superior, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centro_superior = None
    if len(contornos_superior) > 0:
        contorno_mas_grande_superior = max(contornos_superior, key=cv2.contourArea)
        momentos_superior = cv2.moments(contorno_mas_grande_superior)
        if momentos_superior["m00"] != 0:
            centro_superior = (int(momentos_superior["m10"] / momentos_superior["m00"]),
                               int(momentos_superior["m01"] / momentos_superior["m00"]))

    contornos_inferior, _ = cv2.findContours(imagen_borde_inferior, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centro_inferior = None
    if len(contornos_inferior) > 0:
        contorno_mas_grande_inferior = max(contornos_inferior, key=cv2.contourArea)
        momentos_inferior = cv2.moments(contorno_mas_grande_inferior)
        if momentos_inferior["m00"] != 0:
            centro_inferior = (int(momentos_inferior["m10"] / momentos_inferior["m00"]),
                               int(momentos_inferior["m01"] / momentos_inferior["m00"]))

    return centro_superior, centro_inferior

def process_frame(frame):
    global bridge, centroides_pub, angle_pub, image_pub, nose_pub

    roi_top = 380
    roi_bottom = 580

    roi = frame[roi_top:roi_bottom, :]

    centro_superior, centro_inferior = obtener_centro_areas_negras(roi)

    if centro_superior is not None:
        centro_superior = (centro_superior[0], centro_superior[1] + roi_top)
        cv2.circle(frame, centro_superior, 5, (0, 255, 0), -1)

    if centro_inferior is not None:
        mitad_altura = roi_bottom - roi_top
        centro_inferior = (centro_inferior[0], centro_inferior[1] + roi_top + mitad_altura - 30)
        cv2.circle(frame, centro_inferior, 5, (0, 255, 0), -1)

        # Calcular el angulo
        dx = centro_superior[0] - centro_inferior[0]
        dy = centro_superior[1] - centro_inferior[1]

        if abs(dx) < 1 and abs(dy) < 1:
            angulo_grados = 0
        else:
            angulo_radianes = math.atan2(dy, dx)
            angulo_grados = math.degrees(angulo_radianes)

            # Ajustar el angulo para que este en el rango de 0 a 360 grados
            angulo_grados %= 360
            angulo_grados = angulo_grados - 270

        cv2.putText(frame, "Angulo: {:.2f} grados".format(angulo_grados), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Dibujar linea que une los centroides
        cv2.line(frame, centro_inferior, centro_superior, (204, 204, 255), 2)

        # Publicar los centroides en el topico "/centroides"
        centroides_msg = PointStamped()
        centroides_msg.header.stamp = rospy.Time.now()
        centroides_msg.header.frame_id = "camera_frame"
        centroides_msg.point.x = centro_inferior[0]
        centroides_msg.point.y = centro_inferior[1]

        centroides_pub.publish(centroides_msg)

        # Publicar el angulo en el topico "/ang_err"
        nose_pub.publish(6.1)
        if angulo_grados <= 45 and angulo_grados >= -45:
            angle_pub.publish(angulo_grados)
            
        #angle_pub.publish(angulo_grados)
        nose_pub.publish(5.1)

    # Publicar la imagen con los dibujos en el topico "/image_with_detections"
    image_with_detections_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    image_pub.publish(image_with_detections_msg)

def image_callback(msg):
    global bridge

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    process_frame(frame)

def main():
    global bridge, centroides_pub, angle_pub, image_pub, nose_pub, thresh_pub

    rospy.init_node("image_processing_node")
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/video_source/raw", Image, image_callback)
    centroides_pub = rospy.Publisher("/centroides", PointStamped, queue_size=1)
    angle_pub = rospy.Publisher("/ang_err", Float32, queue_size=1)
    
    nose_pub = rospy.Publisher("/nose", Float32, queue_size=1)
    image_pub = rospy.Publisher("/image_with_detections", Image, queue_size=1)
    thresh_pub = rospy.Publisher("/thresh", Image, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
'''
