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

En primer lugar, para el controlador en lazo cerrado se usó un controlador PI para la parte angular y un controlador P para la parte lineal, esto debido a la adaptabilidad que tiene en el sistema que estamos controlando, además de la robustez que tiene para manejar perturbaciones sin que su rendimiento se vea tan afectado.

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


### path_generator.py
Este código genera la trayectoria a seguir por el robot, esto se logra mediante mensajes customizados al tener cada una de las coordenadas en "x" y en "y" por separado. Se comienza por cargar cada una de las librerías y los mensajes customizados correspondientes. Posteriormente se generan funciones de callback que nos serán útiles para conocer los valores de las velocidades que poseen cada una de las llantas. También se hace uso de la biblioteca "Pose2D", esto con el objetivo de conocer la ubicación del robot en todo momento, lo cual nos permitirá hacer un cálculo que nos indicar la posición siguiente que el robot deberá de tomar. Seguido de esto tenemos la creación de los publishers y suscribers correspondientes. Dentro de la función principal para correr el nodo de Ros, tenemos en el desarrollo del código un apartado en el cuál generamos nuestro tiempo de muestreo a partir de las diferencias de tiempo utilizando "Rospy.get_time()". Seguido de esto calculamos el error angular y lineal tmando como referencia la coordenada a la cual se quiere llegar. Y si el error en cierto momento es menor a 0.1 indicamos que el error se convierta en 0 para así pasar al siguiente punto. Este mismo error le servirá al controlador para ayudar al robot a detenerse cuando la distancia sea la requerida. Finalmente se publica la ubicación del robot y el error presente, tanto lineal como angular.


`````python

#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from mini_challenge_3.msg import errores
from geometry_msgs.msg import Pose2D
import numpy as np

class path_generator_class:
    def __init__(self):
      self.current_time = 0.0
      self.previous_time = 0.0
      self.first = True
      self.i = 0
      self.paso = 0
      #sample_time = 0.1
      #tiempo_muestreo = 0.1
      self.input_path_x = 0
      self.input_path_y = 0

      self.wl = 0.0
      self.wr = 0.0

      self.r = 0.05
      self.l = 0.19
      self.first = True

      self.pose = Pose2D()
      self.pose.x = 0.0
      self.pose.y = 0.0
      self.pose.theta = 0.0

      self.el_errores = errores()
      self.el_errores.error_distancia = 0.0
      self.el_errores.error_angular = 0.0

      rospy.init_node("Path_generator")
      self.rate = rospy.Rate(20)
          #pub = rospy.Publisher("/camino", camino, queue_size=10)
      
      rospy.Subscriber("/wr", Float32, self.wr_callback)
      rospy.Subscriber("/wl", Float32, self.wl_callback)
      #self.input_path_x = rospy.get_param("/path_x", [2.0,2.0,0.0,0.0])
      #self.input_path_y = rospy.get_param("/path_y", [0.0,2.0,2.0,0.0])
      self.input_path_x = rospy.get_param("/path_x")
      self.input_path_y = rospy.get_param("/path_y")
      self.el_errores_pub = rospy.Publisher("/Err", errores, queue_size=1)
      self.pose_publicador = rospy.Publisher("/pose",Pose2D,queue_size=1)
      print("The Controller is Running")  

      self.rate = rospy.Rate(20)
      #self.loop_rate = rospy.Rate(20)
      #rospy.on_shutdown()

    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data

    def wrapTheta(self,theta):
        result=np.fmod((theta+ np.pi),(2*np.pi))
        if(result<0):
            result += 2 * np.pi
        return result - np.pi

    def run(self):
        while not rospy.is_shutdown():
            # Compute time since last main loop
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
                #for i in range (0,len(input_path_x)):
                #  path_x.append([path_x[i]])
                #  path_y.append([path_y[i]])
                # self.previous_time = self.current_time

            else:
                self.current_time = rospy.get_time()
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time
    
                
                self.pose.theta += self.wrapTheta(dt * self.r * ((self.wr - self.wl) / self.l))
                self.pose.x += dt * self.r * ((self.wr + self.wl) / 2) * np.cos(self.pose.theta)
                self.pose.y += dt * self.r * ((self.wr + self.wl) / 2) * np.sin(self.pose.theta)
                
                ###############################################################################
                
                self.el_errores.error_angular = self.wrapTheta(np.arctan2(self.input_path_y[self.paso]-self.pose.y, self.input_path_x[self.paso]-self.pose.x) - self.pose.theta)
                self.el_errores.error_distancia = np.sqrt(np.square(self.input_path_x[self.paso]-self.pose.x) + np.square(self.input_path_y[self.paso]-self.pose.y))
               
                if self.el_errores.error_distancia < 0.1:
                    self.el_errores.error_distancia = 0
                if self.el_errores.error_angular < 0.1 and self.el_errores.error_angular > -0.1:
                    self.el_errores.error_angular = 0
                if self.el_errores.error_angular == 0 and self.el_errores.error_distancia == 0 and self.paso < len(self.input_path_x):
                    self.paso += 1
                    print("Se envia otra coordenada")
                if self.i == len(self.input_path_x):
                    print("Llegamos al limite de puntos") 
                    self.el_errores.error_distancia = 0
                    self.el_errores.error_angular = 0
                self.pose_publicador.publish(self.pose)
                self.el_errores_pub.publish(self.el_errores)
                self.rate.sleep()

if __name__ == "__main__":
    Path = path_generator_class()
    try:
        Path.run()
    except rospy.ROSInterruptException:
        None

`````

### controller.py

Este código integra el controlador PI para la parte angular y el controlador P para la parte lineal. Primeramente se cargan las librerías a utilizar, así como los mensajes cutomizados correspondientes. Seguido de esto se declaran las variables a utilizar y las constantes del controlador. Luego de esto se inicializan tanto los suscriptores como los publicadores, es en esta parte donde se suscribe al tópico de "/Err" con la finalidad de obtener el valor del error en el momento preciso, esto servirá para realizar nuestro controlador. Además dentro del mismo apartado tenemos el suscriptor al tópico de "traffic_light" el cual recibirá un dato tipo String dependiendo de si se detecta un color verde, rojo o amarillo proveniente del semáforo. Dentro de la parte más robusta del código encontramos al controlador PI para la velocdad angular y el controlador P para la velocidad lineal, además se tienen condicionales que ayudarán al movimiento del robot, si alguna de las condicionales se cumple el controlador entrar en acción para tratar de corregir la trayectoria. Es en la parte final donde implementamos condicionales que nos ayudarán a mover o detener elrobot dependiendo de color presente en el semáforo, si se detecta el color verde podemos mover el robot, si se detecta el color amarillo reducimos la velocidad por un factor de 0.1 hasta detenerse y si se detecta el color rojo la velocidad del robot será de 0, haciendo que se detenga. Por último se publica la velocidad al tópico de "/cmd_vel".

`````python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import Pose2D
from mini_challenge_3.msg import errores

class controller:
    def __init__(self):
        
        self.tiempo_muestreo = 0.1

        self.r = 0.05
        self.l = 0.19

        self.first = True

        self.estado = " "
        #self.estado_2 = "Ejecuta"
        

        # Variables para el almacenamiento de las posiciones del robot
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        #Variables para alamcenar los tiempos
        self.current_time = 0.0
        self.previous_time = 0.0
        #self.el_errores = errores()
        #self.el_errores.error_distancia = 0.0
        #self.el_errores.error_angular = 0.0

        self.error1 = 0.0
        self.error2 = 0.0


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

        self.kp_linear = 0.35
        self.ki_linear = 0.0
        self.kd_linear = 0.0

        #Variables para cada constante del PID
        self.P_linear = 0.0
        self.I_linear = 0.0
        self.D_linear = 0.0

        self.P_angular = 0.0
        self.I_angular = 0.0
        self.D_angular = 0.0

        #Respuesta del controlador PID
        self.respuesta_linear = 0.0
        self.respuesta_angular = 0.0

        #Acum del error
        self.integral_angular = 0.0
        self.integral_linear = 0.0


   

        #Inicializar nodos
        rospy.init_node("controller")
        #rospy.Subscriber("/wr", Float32, self.wr_callback)
        #rospy.Subscriber("/wl", Float32, self.wl_callback)
        #rospy.Subscriber("/camino", camino, self.camino_callback)
        rospy.Subscriber("/traffic_light", String, self.vision_callback)   
        
        rospy.Subscriber("/Err", errores, self.callback_error)
        
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
        self.error1 = msg.error_distancia
        self.error2 = msg.error_angular
        
    def stop(self):
        print("Stopping")
        self.speed.linear.x = 0
        self.speed.linear.y = 0
        self.speed.linear.z = 0
        self.speed.angular.x = 0
        self.speed.angular.y = 0
        self.speed.angular.z = 0

    #def wrapTheta(self,theta):
    #    result=np.fmod((theta+ np.pi),(2*np.pi))
    #    if(result<0):
    #        result += 2 * np.pi
    #    return result - np.pi

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
                    
                    #dt = 0.00000000001
                    ##################################################################################
                    
                #Controlador_lineal
                self.prev_Error_linear = 0.0
                # P_linear
                self.P_linear = self.kp_linear * self.error1

                # I_linear
                self.I_linear += self.error1 * dt

                # D_linear
                self.D_linear = ((self.error1 - self.prev_Error_linear)/(dt+0.000000001))

                self.prev_Error_linear = self.error1

                self.respuesta_linear = self.P_linear + self.I_linear * self.ki_linear + self.kd_linear * self.D_linear #Corregir
                    ##################################################################################

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
                    self.speed.linear.x = self.respuesta_linear
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
### Ajuste del controlador
El primer paso fue definir un conjunto de valores iniciales para los dos parámetros del controlador PI: la ganancia proporcional (Kp) y la ganancia integral (Ki). Para hacer esto, se utilizaron valores comunes de referencia para cada uno de los parámetros, pero se ajustaron según el comportamiento que se iba observando en el robot.

Luego, se inició el proceso de ajuste manual de los parámetros. Para hacer esto, se aumentó el valor de la ganancia proporcional (Kp) hasta que se observó una oscilación en el sistema. Luego, se disminuyó ligeramente el valor de Kp hasta que la oscilación mejoró. Este valor se registró como el valor inicial de Kp, solamente que notamos una mejor estabilidad en el sistam cuando la ganacia derivativa no estaba presente.

Finalmente, se ajustó el valor de la ganancia integral (Ki) para mejorar la precisión del sistema en estado estable. Se aumentó Ki hasta que se observó una disminución significativa en el error en estado estable, y luego se disminuyó ligeramente Ki hasta que el error en estado estable se mantuvo constante.

Este proceso se repitió varias veces, con pequeños ajustes en cada iteración.

## Resultados

### Video demostrativo

https://drive.google.com/file/d/19pZrLs5qfI0KTE--0tvYzdW7VM33KknC/view?usp=sharing

## Conclusiones

Al trabajar con esta nuevo tema de visión por computadora, se presentó un reto al ajustar las configuraciones de la detección de colores de la cámara de nuestra laptop a la cámara del robot, ya que eran de diferente calidad. Al poder visualizar la máscara de los colores se presentó ruido, por lo que se decidió usar funcions morfológicas y un filtro, después se presentó el problema de que la cámara detectaba colores de objetos a lo lejos, por lo que se programó una condición que involucraba la función cv2.findcontours(), la cual ayudaba a calcular el área del color detectado, si el objeto tenia una área grande, es decir que estaba lo suficientemente cerca, se mandaba el mensaje correspondiente.
Como adición, en este reto se implementó por primera vez el método de incluir el espacio de trabajo con todos los archivos directamente en la Jetson, con el fin de un procesamiento más rápido y eficiente, tuvimos que depender de un monitor, teclado y mouse ó en su defecto estar conectados con múltiples terminales con el protocolo SSH, sin emnargo, notamos mejoras al aprovechar los recursos de la Jetson en los resultados del reto.
