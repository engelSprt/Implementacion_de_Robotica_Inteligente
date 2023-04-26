# CHALLENGE 1

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>


**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3002B.502.</p>
<p align="center"> Implementación de robótica inteligente (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 1</p>
<p align="center"> Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos		A01735589</p>
<p align="center">José Ángel Ramírez Ramírez		A01735529</p>
<p align="center">Daniel Ruán Aguilar			A01731921</p>
<p align="center">Fecha de entrega: 18 de abril del 2023</p>


## Resumen

En este reto se refuerzan los conocimientos vistos durante la primera semana que consistía en hacer pruebas con la comunicación del robot que se estará utilizando durante la unidad de formación (puzzlebot). 

Para manipular el robot desde otra computadora se ocupa el protocolo SSH y exportar el ROS master, de esta manera se puede programar los nodos desde nuestra computadora para lograr el objetivo de este primer challenge sin la necesidad de un monitor para la Jetson nano, la cual será la computadora integrada del robot.

En adición la ejecución del robot en físico se hará al mismo tiempo en el simulador de ROS Gazebo, ya que es importante aprender a utilizarlo en caso de que no se tenga el robot para trabajar. Este simulador permite probar y depurar el código antes de implementarlo en un robot físico y de esta manera evitar errores.

## Objetivos

Lo que se busca es crear un nodo para conducir el robot simulado en una ruta cuadrada de una longitud de 2 m por lado. Al mismo tiempo se usa el mismo nodo para mover el robot real en un cuadrado de 2 m de lado utilizando un controlador de lazo abierto y se selecciona la velocidad para terminar el recorrido, finalmente se trazarán segmentos de distancia que el robot rescorrerá tanto en físico y en el simulador.

**Trayectoria 1**
![image](https://user-images.githubusercontent.com/93226207/234476444-65daa0d9-1407-4499-a098-584d89ef6d62.png)


## Introducción

La inclusión de un robot movil al trabajar con R.O.S. (Robot Operating System) require la manipulación e investigación de diferentes herramientas nuevas para la comunicación entre la jetson nano y otra computadora externa. Las herramientas empleadas en este primer reto son las siguientes.

- Gazebo
- Puzzlebot
- Comunicación SSH
- Ros Master URI

<p align="center">
  <img src="https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/blob/main/challenge1/Challenge1_brainstorm.png" />
</p>

## Solución del problema
Para la solución de este reto se cuentan con dos archivos codificados en lenguaje Python, llamados: avanza.py y trayectoria2.py Se comenzará por describir la funcionalidad de cada uno en el siguiente apartado:

### cuadrado.py
Este nodo controla al puzzlebot para que se mueva en un patrón de cuadrado de 2mts por 2mts. El código se ejecuta en un bucle continuo hasta que complete su trayectoria o el usuario lo detenga. Durante el bucle, el nodo utiliza información de velocidad de rueda proporcionada por dos suscriptores /wr y /wl, para calcular la distancia y el ángulo recorridos por el puzzlebot. Luego, el nodo utiliza esta información para determinar la dirección y la velocidad de movimiento del robot. Finalmente, el nodo publica los comandos de movimiento a través de un publicador de ROS al robot.

La clase Square define la funcionalidad del nodo. La función __init__ inicializa el nodo y los objetos de publicación y suscripción de ROS. Los valores iniciales para las velocidades de las ruedas son cero. También se establece una tasa de publicación de 20 Hz.

La función wr_callback y wl_callback se utilizan para actualizar las velocidades de las ruedas. Estas funciones se llaman automáticamente cuando los valores de velocidad de las ruedas son publicados en los tópicos de ROS /wr y /wl.

La función run contiene el bucle principal del nodo. El bucle se ejecuta continuamente hasta que el usuario lo detiene. En cada iteración del bucle, el nodo calcula la distancia y el ángulo recorridos por el robot desde la última iteración utilizando la velocidad de las ruedas. Luego, el nodo determina la posición del robot y publica los comandos de movimiento necesarios para mover el robot al siguiente punto en el patrón de cuadrado.

La función stop se utiliza para detener el robot de manera segura cuando se detiene el nodo. Se publica un mensaje de parada en el tópico de ROS /cmd_vel.

`````python   

#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

class Square:
    def __init__(self):
        rospy.init_node("square")
        
        # Initialise wheel velocity variables
        self.wr = 0.0
        self.wl = 0.0

        # Setup ROS subscribers and publishers
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)
        self.w_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.rate = rospy.Rate(20) # Set the publishing rate

    # Callbacks for wheel velocities and commands
    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    # Main function
    def run(self):
        # Variable initialisations
        distance = 0.0
        angle = 0.0

        # Create message for publishing
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # Main Loop
        pos = 1
        cont = 0
        freq = 25.0
        rate = rospy.Rate(freq)
        dt = 1/25.0
        while not rospy.is_shutdown():
            # Compute time since last loop
           

            # Update distance and angle from the velocity measurements
            distance += 0.05 * (self.wr + self.wl) * 0.5 * dt
            angle += 0.05 * (self.wr - self.wl) / 0.18 * dt
            self.wr = 0
            self.wl = 0

            # Set robot motion
            if pos == 1:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                self.w_pub.publish(msg)

                # Check if distance is reached
                if distance > 2:
                    distance = 0.0
                    pos = 2

            elif pos == 2:
                msg.linear.x = 0.0
                msg.angular.z = 0.2
                self.w_pub.publish(msg)

                # Check if angle is reached
                if angle > 3.020/2:
                    if cont < 3:
                        pos = 1
                        cont += 1
                        angle = 0.0
                    else:
                        pos = 3

            elif pos == 3:
                # Stop robot and print motion completed message
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.w_pub.publish(msg)
                rospy.loginfo("Motion Completed")
                rospy.signal_shutdown("Square Completed")

            rate.sleep()

    # Separate stop function for stopping when ROS shuts down
    def stop(self):
        rospy.loginfo("Stopping")
        msg = Twist()
        self.w_pub.publish(msg)

if __name__ == "__main__":
    sq = Square()

    try:
        sq.run()
    except rospy.ROSInterruptException:
        pass
`````


### trayectoria2.py
Este nodo de controla el puzzlebot para que se mueva en una trayectoria similar segunda trayectoria propuesta. El programa se suscribe a los tópicos '/wr' y '/wl', que representan la velocidad angular de las ruedas derecha e izquierda, respectivamente. Luego, utiliza estas velocidades para calcular la distancia y el ángulo recorridos por el robot. Además, utiliza la biblioteca NumPy para definir cuatro ángulos a través de los cuales el robot girará y avanzará en línea recta para crear el patrón de cuadrado.

Después de inicializar las variables y los tópicos, el programa entra en un bucle principal. En este bucle, utiliza una variable de posición para determinar la etapa en la que se encuentra el robot, y establece las velocidades lineales y angulares correspondientes para cada posición. Las posiciones están numeradas del 0 al 8, donde la posición 0 es el punto de partida y la posición 8 es el punto final. Cada posición corresponde a un movimiento específico del robot.

Primero, el robot gira a la derecha hasta que alcanza un ángulo de 30 grados. Luego avanza 2 metros en línea recta. A continuación, gira a la izquierda 110 grados y avanza otros 2 metros en línea recta. Luego, gira a la derecha 90 grados y avanza 2 metros en línea recta. Finalmente, gira a la izquierda 80 grados y avanza 3 metros en línea recta para completar el patrón de cuadrado.

El bucle se repite hasta que se interrumpe el programa o se alcanza la posición final. Cuando se alcanza la posición final, el robot se detiene y se muestra un mensaje en la consola de que la tarea se ha completado. Si el programa se interrumpe por cualquier motivo, se detiene el robot.

`````python  
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

class Square:
    def __init__(self):
        rospy.init_node("square")

        # Initialise wheel velocity variables
        self.wr = 0.0
        self.wl = 0.0

        # Setup ROS subscribers and publishers
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)
        self.w_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.rate = rospy.Rate(20) # Set the publishing rate

    # Callbacks for wheel velocities and commands
    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    # Main function
    def run(self):
        # Variable initialisations
        distance = 0.0
        angle = 0.0

        # Create message for publishing
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # Main Loop
        pos = 0
        cont = 0
        freq = 25.0
        rate = rospy.Rate(freq)
        dt = 1/25.0
        a1 = np.pi/6 #30 
        a2 = (11*np.pi)/18 #110
        a3 = (10*np.pi)/9 #200
        a4 = (14*np.pi)/9 #280

        while not rospy.is_shutdown():
            # Compute time since last loop


            # Update distance and angle from the velocity measurements
            distance += 0.05 * (self.wr + self.wl) * 0.5 * dt
            angle += 0.05 * (self.wr - self.wl) / 0.18 * dt
            self.wr = 0
            self.wl = 0

            # Set robot motion
            if pos == 0:
                msg.linear.x = 0.0
                msg.angular.z = 0.2
                self.w_pub.publish(msg)

                if angle >= a1:
                    angle = 0.0
                    pos = 1

            elif pos == 1:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                self.w_pub.publish(msg)

                # Check if distance is reached
                if distance > 2:
                    distance = 0.0
                    pos = 2


            elif pos == 2:
                msg.linear.x = 0.0
                msg.angular.z = 0.2
                self.w_pub.publish(msg)

                if angle >= a2:
                    angle = 0
                    pos = 3

            elif pos == 3:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                self.w_pub.publish(msg)

                # Check if distance is reached
                if distance > 2:
                    distance = 0.0
                    pos = 4

            elif pos == 4:
                msg.linear.x = 0.0
                msg.angular.z = 0.2
                self.w_pub.publish(msg)

                if angle >= a3:
                    angle = 0
                    pos = 5

            elif pos == 5:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                self.w_pub.publish(msg)

                # Check if distance is reached
                if distance > 2:
                    distance = 0.0
                    pos = 6

            elif pos == 6:
                msg.linear.x = 0.0
                msg.angular.z = 0.2
                self.w_pub.publish(msg)

                if angle >= a4:
                    angle = 0
                    pos = 7

            elif pos == 7:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                self.w_pub.publish(msg)

                # Check if distance is reached
                if distance > 3:
                    distance = 0.0
                    pos = 8

            elif pos == 8:
                # Stop robot and print motion completed message
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.w_pub.publish(msg)
                rospy.loginfo("Motion Completed")
                rospy.signal_shutdown("Square Completed")

            rate.sleep()

    # Separate stop function for stopping when ROS shuts down
    def stop(self):
        rospy.loginfo("Stopping")
        msg = Twist()
        self.w_pub.publish(msg)

if __name__ == "__main__":
    sq = Square()

    try:
        sq.run()
    except rospy.ROSInterruptException:
        pass

`````
## Resultados  

**<p align="center"> Videos de demostración en el robot</p>**

**Trayectoria Cuadrada**

https://youtube.com/shorts/2Tvn88L7sOc?feature=share

**Trayectoria definida por segmentos**

https://youtu.be/BI_4bEUJZ0A

**<p align="center"> Videos de demostración del robot en el simulador Gazebo</p>**

**Trayectoria Cuadrada**

https://youtu.be/MvmP8JJIncQ

**Trayectoria definida por segmentos**

https://youtu.be/QfGIKjejXoo

En estos videos se puede ver como el robot realiza trayectorias tanto en físico y en el simulador.

## Conclusiones

Consideramos que esta primera práctica nos ayudó a terminar de comprender cómo funcionan las herramientas básicas de comunicación del Puzzlebot y ROS, las cuales son importantes e indispensables para desarrollar los futuros retos del curso que implementarán más factores, como la visión por computadora y el control de lazo cerrado. Pudimos resolver los erroes e impedimentos que tuvo el robot de las primeras pruebas en la semana, lo cual nos quitó un poco de tiempo en la resoluión de este reto, sin embargo, pudimos resolver el reto planteado, que en este caso fue la generación de trayectorias con el robot.

