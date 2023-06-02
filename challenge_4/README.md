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
Para la solución de este reto se cuentan con 3 archivos codificados en lenguaje Python, llamados: semaphore.py controlador_prueba.py, follow_line.py, follow_line2.py. Se comenzará por describir la funcionalidad de cada uno en el siguiente apartado:

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
            if area_Red >5000 and cv2.countNonZero(mask_red) > 1000:
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
            if area_yellow >5000 and cv2.countNonZero(mask_yellow) > 1000:
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
            if area_green >5000 and cv2.countNonZero(mask_green) > 1000:
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

### controlador_prueba.py
El controlador utiliza el framework ROS para la comunicación con otros nodos y la suscripción a tópicos relevantes. Se suscribe a los tópicos "/traffic_light" y "/line_error" para recibir información sobre el estado del semáforo y el error de línea detectado, respectivamente. El error de la línea no es más que el ángulo que se detectó con el programa de visión, dicho ángulo entrará como set_point al controlador y este generará al final una velocidad utilizando la instrucción de Twist con la velocidad controlada con ayuda del controlador PID.

El controlador está diseñado como una clase llamada "controller", que encapsula todas las funcionalidades necesarias. Algunas de las variables clave utilizadas incluyen el tiempo de muestreo, las dimensiones físicas del robot, el estado actual, las ganancias del controlador PID y las velocidades lineal y angular.

Dentro del bucle principal, el controlador implementa un algoritmo de control PID para determinar las velocidades deseadas del robot. Calcula la respuesta del controlador PID utilizando el error angular y las ganancias establecidas. Dependiendo del estado del semáforo, el controlador ajusta las velocidades lineal y angular del robot para realizar diferentes acciones.

Si el estado del semáforo es "Rojo", el controlador detiene completamente el robot estableciendo las velocidades lineal y angular en cero. Esto garantiza que el robot se detenga de manera segura cuando el semáforo está en rojo y no avance hasta que cambie la señal.

Cuando el estado del semáforo es "Verde", el controlador permite que el robot avance ajustando las velocidades lineal y angular según las respuestas del controlador PID. Estas velocidades se calculan para lograr un movimiento suave y controlado.

En el caso de que el estado del semáforo sea "Amarillo", el controlador reduce gradualmente las velocidades lineal y angular del robot. Esto se logra disminuyendo los valores de las velocidades en un factor predefinido, lo que permite al robot frenar suavemente antes de detenerse por completo. La reducción gradual de las velocidades evita cambios bruscos y brinda una transición segura cuando el semáforo pasa de verde a rojo.

El controlador publica las velocidades controladas en el tópico "/cmd_vel" para que el robot las reciba y las ejecute. Esto permite una comunicación efectiva entre el controlador y el sistema de control del robot.

En general, este controlador implementado en ROS permite al robot operar de manera autónoma, respondiendo a señales externas y ajustando su movimiento según un algoritmo de control PID. Proporciona flexibilidad y modularidad al separar la lógica del controlador del resto del sistema, lo que facilita el desarrollo y la depuración de aplicaciones robóticas más complejas.


`````python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64
#from geometry_msgs.msg import Pose2D
from retosemaforo.msg import errores # modificar el paquete modificar el custom message

class controller:
    def __init__(self):
        
        self.tiempo_muestreo = 0.1

        self.r = 0.05
        self.l = 0.19

        self.first = True

        self.estado = " "
        #self.estado_2 = "Ejecuta"
        

        # Variables para el almacenamiento de las posiciones del robot
        #self.pose = Pose2D()
        #self.pose.x = 0.0
        #self.pose.y = 0.0
        #self.pose.theta = 0.0

        #Variables para alamcenar los tiempos
        self.current_time = 0.0
        self.previous_time = 0.0
        #self.el_errores = errores()
        #self.el_errores.error_distancia = 0.0
        #self.el_errores.error_angular = 0.0

        self.error1 = 0.0

        
        #Declaracion de las variables reales para el calculo del angulo
        self.pos_real_x = 0.0
        self.pos_real_y = 0.0



        # Initialize Twist message for robot speed
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0

        #self.pwm = 0


        # Initialize PID controller gains and integrals
        self.kp_angular = 0.5
        self.ki_angular = 0.1
        self.kd_angular = 0.1

        #self.kp_linear = 0.35
        #self.ki_linear = 0.0
        #self.kd_linear = 0.0

        #Variables para cada constante del PID
        #self.P_linear = 0.0
        #self.I_linear = 0.0
        #self.D_linear = 0.0

        self.P_angular = 0.0
        self.I_angular = 0.0
        self.D_angular = 0.0

        #Respuesta del controlador PID
        #self.respuesta_linear = 0.0
        self.respuesta_angular = 0.0

        #Acum del error
        self.integral_angular = 0.0
        #self.integral_linear = 0.0



        #Inicializar nodos
        rospy.init_node("controller")
        #rospy.Subscriber("/wr", Float32, self.wr_callback)
        #rospy.Subscriber("/wl", Float32, self.wl_callback)
        #rospy.Subscriber("/camino", camino, self.camino_callback)
        rospy.Subscriber("/traffic_light", String, self.vision_callback)   
        
        rospy.Subscriber("/line_error", Float64, self.vision_callback)
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        #self.semaforo_estado = rospy.Publisher('/cmd_vel', UInt8, queue_size = 10)

        self.rate = rospy.Rate(20)

        # Callbacks para velocidades de las ruedas
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

    def vision_callback(self,data):
        if data.data == "Stop":
            self.estado = "Rojo"
        elif data.data == "Slow_down":
            self.estado = "Amarillo"
        elif data.data == "Go":
            self.estado = "Verde"
        else:
            return
    
    def callback_error(self,msg):
        self.error1 = msg.error_angular

    #def callback_posicion(self,msg):
    #    self.error1 = msg.error_angular
        
    def stop(self):
        print("Stopping")
        self.speed.linear.x = 0
        self.speed.linear.y = 0
        self.speed.linear.z = 0
        self.speed.angular.x = 0
        self.speed.angular.y = 0
        self.speed.angular.z = 0

    def wrapTheta(self,theta):
        result=np.fmod((theta+ np.pi),(2*np.pi))
        if(result<0):
            result += 2 * np.pi
        return result - np.pi

    def run(self):
        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time() 
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time

                    #self.semaforo_estado.publish(self.pwm)
                    #if dt >= self.tiempo_muestreo:
                    # Actualizar las posiciones
                    #dt = 0.00000000001
                    ##################################################################################
                    
                #self.el_errores.error_angular = self.wrapTheta(np.arctan2(self.input_path_y[self.paso]-self.pose.y, self.input_path_x[self.paso]-self.pose.x) - self.pose.theta)

                #Controlador_angular
                self.prev_Error_angular = 0.0
                # P_angular
                self.P_angular = self.kp_angular * self.error2

                # I_angular
                self.I_angular += self.error2 * dt

                # D_angular
                self.D_angular = ((self.error2 - self.prev_Error_angular)/(dt+0.0000001))

                self.prev_Error_angular = self.error2

                self.respuesta_angular = self.P_angular + self.I_angular * self.ki_linear + self.kd_angular * self.D_angular                
                
                ##########################self.current_time = rospy.get_time()########################################################
                
                if self.error1 == 0 or self.error2 == 0:
                    #Publicar las posiciones
                    #self.err_pub.publish(self.el_errores)
                    self.speed.linear.x = 0
                    self.speed.angular.z = 0

            # Semaforo en ROJO -> Se detiene el robot.

                if self.estado == "Rojo":
                    self.speed.linear.x = 0
                    self.speed.angular.z = 0
                    #print("Si estoy publicando las velocidades controladas \n")
                    #self.vel_pub.publish(self.speed)                   

            # Semaforo en Verde -> Avanza el robot.

                elif self.estado == "Verde":

                    self.speed.linear.x = self.respuesta_linear
                    self.speed.angular.z = self.respuesta_angular
                    #print("Si estoy publicando las velocidades controladas \n")
                
                elif self.estado == "Amarillo":
                    self.speed.linear.x -= 0.1
                    self.speed.angular.z -= 0.1
                    if self.speed.linear.x <= 0 or self.speed.angular.z <= 0:
                    # si la velocidad es cero, detener el robot
                        self.stop()
                
                else:
                    self.speed.linear.x = 0.1
                    self.speed.angular.z = self.respuesta_angular
                    #print("NO_hago_nada")

                self.vel_pub.publish(self.speed)
                print("Si estoy publicando las velocidades controladas \n")
                self.rate.sleep()
                
if __name__ == "__main__":
    Controller = controller()
    try:
        Controller.run()
    except rospy.ROSInterruptException:
        None 

`````

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
![image](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/93226207/e699fffc-9db7-4997-8066-a58921681bdd)
![image](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/93226207/2e8de582-2b04-40d7-9d2c-eb297906ae22)
![image](https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/assets/93226207/9855466f-3654-4755-b7c3-68a9e509f0e2)


## Resultados
**Videos de ejecución en pista**  

https://youtube.com/shorts/hFCaqIVo1SE?feature=share

https://youtube.com/shorts/tg0sDvfeq4Y?feature=share

**Video de explicación de código de visión computacional**

https://youtu.be/jnjquMFUAJ8

**Video de demostración de segundo programa de visión**

https://youtu.be/xkm3niGPm3Y

https://youtu.be/y_p1Pb6AsKM
 
## Conclusiones

Un problema que se presentó fue al momento de pasar nuestros códigos probados en nuestros ordenadores a la jetson, ya que la cámara del puzzlebot es de una menor calidad que las de las computadoras, por lo que no segmentaba de igual manera las líneas negras, entonces se tuvo que modificar e ir cambiando las variables dentro de la jetson para acoplarse a las nuevas condiciones de la cámara-raspberry pi. En adición a esto, hubo problemas con la detección de la línea dependiendo de la iluminación que hubiera en el espacio de experimentación, entre más luz había, menor era el umbral que se debía aplicar y con menos luz, debía declararse uno mayor, es por esto que consideramos como una posible mejora al código la regularización automática de este umbral dependiendo de la luz en el entorno.

Otro inconveniente que hubo para la realización del reto, fue durante la semana anterior, que por causas naturales en el estado de Puebla, se cambió la modalidad de un día para otro de presencial a remoto, lo cual dificultó la fluidez del trabajo colaborativo, además de que el robot y la guía para crear la pista se habían quedado en el campus. Sin embargo, consideramos que se logró el objetivo principal de este challenge, el cual fue la detección de la línea para seguir la trayectoria deseada.
