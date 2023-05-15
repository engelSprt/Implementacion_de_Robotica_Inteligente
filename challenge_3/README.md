# CHALLENGE 3

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>

**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3002B.502.</p>
<p align="center">Implementación de robótica inteligente (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 3</p>
<p align="center">Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos     A01735589</p>
<p align="center">José Ángel Ramírez Ramírez    A01735529</p>
<p align="center">Daniel Ruán Aguilar           A01731921</p>
<p align="center">Fecha de entrega: 15 de Mayo del 2023</p>


## Resumen
Este reto es una continuación de los retos 1 y 2. Para este, los puntos más importantes a tomar en cuenta son el control en lazo cerrado y algoritmos de visión. 

En primer lugar, para el controlador en lazo cerrado se usó un controlador PID debido a la adaptabilidad que tiene en el sistema que estamos controlando, además de la robustez que tiene para manejar perturbaciones sin que su rendimiento se vea tan afectado.

Por otro lado, para el sistema de visión, se definió un rango para cada uno de los colores, se usó un filtro "Gaussian Blur" con un kernel de 5x5 para filtrar el ruido de las imágenes, y para agregar robustez se usaron conceptos de morfología con el propósito de que hubiera una visión más detallada de cada color.



## Objetivos
Considerando los conceptos vistos durante la semana 4 del curso, se pretende demostrar el comportamiento de los sistemas de visión computacional aplicado en los robots móviles. 

Particularmente en este mini challenge se busca usar los conocimientos adquiridos en las semanas previas del curso y añadir una capa de toma de decisiones. Esta consiste en añadir un semáforo entre cada punto de una ruta a seguir por el robot como se muestra en la siguiente imagen.

![imagen1](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/100887194/07eb9315-d4d6-4742-8f01-2dfb72fe6270.png)

De tal manera mediante el desarrollo de algoritmos de visión computacional, el robot sea capaz de detectar los distintos tipos de color de cada semáforo y que se comporte de la siguiente manera dependiendo del color:

- Rojo: Detenerse completamente hasta detectar el color verde.
- Amarillo: Avanzar lentamente hasta detectar la luz roja para detenerse. 
- Verde: Continuar con su ruta. 

## Introducción 
Uno de los temas que actualmente están cobrando bastante relevancia en el mundo de la tecnología es la navegación autónoma. 
Por ello, en este mini reto se abordan conceptos que son fundamentales en la misma, de tal manera que se puedan entender los principios básicos y la robustez con que estos deben ser programados a fin de que el robot cumpla su objetivo de la manera más adecuada y correcta posible. Para ello se abordan puntos importantes como:
* Procesamiento de imágenes en una tarjeta embebida 
* Interconexión entre Jetson y la cámara 
* Detección de contornos o formas en una imagen
* Detección de colores 
* Robustez en sistemas de procesamiento de imágenes

![reto 3 mapa mental](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/100887194/8cf8ca51-1edb-4456-ba87-b4caa0b04651)



## Solución
Para la solución de este reto se cuentan con dos archivos codificados en lenguaje Python, llamados: controller.py, semaphore.py y path_generator. Se comenzará por describir la funcionalidad de cada uno en el siguiente apartado:

### semaphore.py
Para la visión por computadora se debe subscribir al tópico de la cámara previamente creado por MCR2 y posterioremente publica en un nuevo tópico nuevo llamado /traffic_light, en el cual manda el mensaje correspondiente por el color detectado, en adición se publica como imagen las máscaras de los tres colores definidos.

Para el algoritmo, primero se definien los rangos para los colores que se buscan detectar, se convierte el fotograma de BGR a HSV y para eliminar el ruido se ocupa un filtro Gaussiano y funciones de morfolgía vistas en el módulo de visión por computadora.

El código comentado en cada una de sus partes significativas queda de la siguiente manera:
`````python 
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
`````

### controller.py

### path_generator.py

## Resultados

### Video demostrativo



## Conclusiones

En cuanto a la parte de visión por computadora, se presentó un reto al ajustar las configuraciones de la detección de colores de la cámara de nuestra laptop a la cámara del robot, ya que eran de diferente calidad. Al poder visualizar la máscara de los colores se presentó ruido, por lo que se decidió usar funcions morfológicas y un filtro, después se presentó el problema de que la cámara detectaba colores de objetos a lo lejos, por lo que se programó una condición que involucraba la función cv2.findcontours(), la cual ayudaba a calcular el área del color detectado, si el objeto tenia una área grande, es decir que estaba lo suficientemente cerca, se mandaba el mensaje correspondiente.
