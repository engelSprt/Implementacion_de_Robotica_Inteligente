# CHALLENGE 4

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>

**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3002B.502.</p>
<p align="center">Implementación de robótica inteligente (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 4</p>
<p align="center">Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos     A01735589</p>
<p align="center">José Ángel Ramírez Ramírez    A01735529</p>
<p align="center">Daniel Ruán Aguilar           A01731921</p>
<p align="center">Fecha de entrega: 28 de Mayo del 2023</p>


## Resumen
Preparándonos para el reto final, esta vez se requirió hacer un nodo adicional para detectar a través de la cámara del Puzzlebot la línea negra de una pista para seguir la trayectoria mediante el control del robot ajustando la velocidad angular y dejando la lineal constante. La pista fue construída por nosotros con materiales de papelería para crear una trayectoría personalizada. La lógica en diagrama a bloques de la programación fue de la siguiente manera:
 
![image](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/blob/main/challenge_4/diagram.png)

## Objetivos

Continuando con la serie de mini-retos semanales, en esta ocasión se nos ha planteado usar nuevamente los conceptos de vison computacional de manera que con base en el mini-reto anterior, seamos capaces de agregar una nueva capa que nos permita hacer seguimiento de lineas usando la camara incluida en el puzzlebot.
Para este mini-reto se espera que el puzzebot sea capaz y tenga la robustez necesaria para seguir una linea sobre una pista, misma que será diseñada por los integrantes del equipo y que debe cumplir con las medidas establecidad por el socioformador en la presentacion de este mini-reto, ademas de esto, el algoritmo debe mantener la deteccion de un semaforo con los distintos colores y la respectiva accion para cada color. 


## Introducción 

Los automóviles autónomos hoy en día han cobrado mucha relevancia y aún hoy en día para muchas personas pareciera que estos funcionan con magia; sin embargo, detrás de estos automóviles existe un tema bastante complejo pero fundamental para que puedan ser autónomos. Es la visión computacional, la cual es un campo de estudio que se enfoca en enseñar a las computadoras a "ver" y comprender el contenido visual de imágenes o videos, permitiéndoles realizar tareas como reconocimiento de objetos, detección de patrones, seguimiento de movimientos y análisis de escenas. En el caso de los vehículos autónomos se tienen que tomar en cuenta distintos puntos, de manera que el sistema trabaje de la manera más eficiente. Algunos de los puntos más importantes para este reto son los siguientes:

* Detección de líneas y contornos en una imagen.
* Algoritmos implementados para el seguimiento de líneas y trayectorias.
* Robustez en sistemas de procesamiento de imágenes, etc.

![mapa mental reto4](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/100887194/f4a8d9a2-09ab-49c0-9422-c32dca103cb1)


## Solución
Para la solución de este reto se cuentan con 3 archivos codificados en lenguaje Python, llamados: controller.py, follow_line.py, follow_line2.py. Se comenzará por describir la funcionalidad de cada uno en el siguiente apartado:

### follow_line.py
En este código se implementa conceptos de visión por computadora, donde se umbraliza lo que capta la cámara para detectar áreas negras en el frame recortado, encuentra el contorno más grande y calcula su centroide. Luego, muestra los contornos, los puntos marcados y el error en el frame recortado y en la imagen binaria, y publica los resultados en varios tópicos. El código documentado queda de la siguiente manera:

`````python 
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
`````

### follow_line2.py
En adición, para una mayor robustez se incorporó otro nodo de visión, donde esta vez mide el error del ángulo de dos puntos en el frame, dicho error se manda de igual manera como un tópico /err_angle para controlar la velocidad angular, el punto inferior del frame indica la coordenada de donde se encuentra el robot y el punto superior indica la coordenada destino a donde se va a ir moviendo el robot. El código comentado queda de la siguiente manera:
![image](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/93226207/e81b8883-ec37-4233-8c50-fc804eafbec7)


## Resultados



## Conclusiones

Un problema que se presentó fue al momento de pasar nuestros códigos probados en nuestros ordenadores a la jetson, ya que la cámara del puzzlebot es de una menor calidad que las de las computadoras, por lo que no segmentaba de igual manera las líneas negras, entonces se tuvo que modificar e ir cambiando las variables dentro de la jetson para acoplarse a las nuevas condiciones de la cámara-raspberry pi. En adición a esto, hubo problemas con la detección de la línea dependiendo de la iluminación que hubiera en el espacio de experimentación, entre más luz había, menor era el umbral que se debía aplicar y con menos luz, debía declararse uno mayor, es por esto que consideramos como una posible mejora al código la regularización automática de este umbral dependiendo de la luz en el entorno.

Otro inconveniente que hubo para la realización del reto, fue durante la semana anterior, que por causas naturales en el estado de Puebla, se cambió la modalidad de un día para otro de presencial a remoto, lo cual dificultó la fluidez del trabajo colaborativo, además de que el robot y la guía para crear la pista se habían quedado en el campus. Sin embargo, consideramos que se logró el objetivo principal de este challenge, el cual fue la detección de la línea para seguir la trayectoria deseada.
