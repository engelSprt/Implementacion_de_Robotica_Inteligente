# CHALLENGE 2

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>


**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3002B.502.</p>
<p align="center"> Implementación de robótica inteligente (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 2</p>
<p align="center"> Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos		A01735589</p>
<p align="center">José Ángel Ramírez Ramírez		A01735529</p>
<p align="center">Daniel Ruán Aguilar			A01731921</p>
<p align="center">Fecha de entrega: 25 de abril del 2023</p>


## Resumen

En este reto se aplican los conocimientos vistos durante la semana, los cuales le dan continuidad al primer reto, pero esta vez mostrando el comportamiento de un control diseñado de lazo cerrado en robótica.

Para lograr la programación de este nuevo comportamiento en el robot se reutilizaron los conceptos requeridos en la implementación del control vistos previamente, tales como el cálculo del error generado, obtener la posición del robot, tipos de controladores, etc. 

## Objetivos

El objetivo consiste en utilizar un control PID para mover el robot a diferentes posiciones en el espacio planteado. Nuevamente se debe conducir el robot en una ruta cuadrada de una longitud de 2 m por lado, posteriormente se se debe crea un nodo generador de rutas, que publique la ruta actual y
próximo objetivo una vez que el robot complete el objetivo actual.

Estas trayectorias se implementan utilizando el Gazebo Puzzlebot Simulator y físicamente en el robot en tiempo real. Para esto se crearán los nodos y paquetes en ROS necesarios que se ecplicarán en la solución del problema.

**Parte 1:**

![image](https://user-images.githubusercontent.com/93226207/234477100-c4fbc079-bf02-458c-84d6-82ce666356a5.png)

**Parte 2:**

![image](https://user-images.githubusercontent.com/93226207/234477041-0a6a4fa0-450d-422f-960e-61db9eed4a5b.png)


## Introducción

A partir del reto 1 que involucraba la movilidad del puzzlebot en lazo abierto, se investigaron y aplicaron los siguientes conceptos para la resolución de este nuevo reto.
- Control en lazo cerrado para un robot móvil
- PID aplicado a un robot móvil diferencial
- Cálculo del error
- Robustez de un controlador

<p align="center">
  <img src="https://github.com/engelSprt/Implementacion_de_Robotica_Inteligente/blob/main/challenge%202/challenge2_brainstorm.png" />
</p>

## Solución del problema
Para la solución de este reto se cuentan con tres archivos codificados en lenguaje Python, llamados: path_generator.py, controller.py y .py Se comenzará por describir la funcionalidad de cada uno en el siguiente apartado:

### path_generator.py

`````python
#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from prueba_reto.msg import goal

# Setup Variables

time = 0
error_theta = 0
error_d = 0
pointSelf = [0,0]

def callbackErrorTheta(msg):
   global error_theta
   error_theta = msg.data

def callbackErrorD(msg):
   global error_d
   error_d = msg.data

#Stop Condition
def stop():
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("path_generator")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers 
    pub = rospy.Publisher("/goal", goal, queue_size = 2)
    pub2 = rospy.Publisher("/balance", Float32, queue_size= 10)
    rospy.Subscriber("/error_theta", Float32, callbackErrorTheta)
    rospy.Subscriber("/error_d", Float32, callbackErrorD)
    goal = goal()
    input_path = rospy.get_param("/path")
    path = []

    check = 0
    step = 0
    balance = 0.0
    time_elapse = 0.0
    

    print("Path Generator is Running")
    

    #Run the node
    while not rospy.is_shutdown():
        if(check == 0):
            for i in range(0,len(input_path)/2):
                path.append([input_path[i*2],input_path[i*2+1]])
            print(path)
            start_time = rospy.get_time()
            time_elapse = start_time
        else:
            time = rospy.get_time() - start_time
            # Update step when error_d is less than 0.1 sec and it has been for at least 0.2 sc
            # then reset balance with a sleep of 0.1 sec
            if(error_d < 0.1 and step < len(path)-1 ):
              balance += rospy.get_time() - time_elapse
              if(balance > 0.2):
                step += 1
                balance = 0.0
                rospy.sleep(0.1)
              start_time = rospy.get_time()
            else:
              balance = 0.0

            time_elapse = rospy.get_time()
               
        
        goal.pos_x = path[step][0]
        goal.pos_y = path[step][1]
        check +=1
      
        pub.publish(goal)
        pub2.publish(balance)
      
      #Write your code here 
      

        rate.sleep()

`````

### controller.py

`````python
#!/usr/bin/env python
import rospy
import numpy as np
#import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from prueba_reto.msg import goal


class puzzlebot:
    def __init__(self):
        # Initialize variables for wheel speeds, robot pose, target pose, and error values
        self.wr = 0.0
        self.wl = 0.0
        self.r = 0.05
        self.l = 0.19
        self.x_k = 0.0
        self.y_k = 0.0
        self.target_x = 0.0
        self.target_y = 0.0 
        self.theta_k = 0.0
        self.error_d = 0.0
        self.error_theta = 0.0
        self.last_error_theta = 0.0
        self.last_error_d = 0.0
        self.last_target = [0,0]


        # Initialize Twist message for robot speed
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0


        # Initialize flags and parameters for control logic
        self.beginning = False
        self.current_step = 0 
        self.check = 0
        self.pause = 0
        self.hold = False
        self.angular_threshold_reached = False
        self.linear_threshold_reached = False


        # Initialize PID controller gains and integrals
        self.kp_angular = 0.5
        self.kp_linear = 0.35
        self.ki_linear = 0.0
        self.ki_angular = 0.0


        self.integral_angular = 0.0
        self.integral_linear = 0.0


        # Initialize ROS publishers for robot speed and pose, and subscribers for wheel speeds and goal pose
        self.main_pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)
        self.pub = rospy.Publisher("position", Pose2D, queue_size = 10)
        self.pub2 = rospy.Publisher("error_theta", Float32, queue_size = 10)
        self.pub3 = rospy.Publisher("error_d", Float32, queue_size = 10)
        self.data_out = Pose2D()
        self.data_out.x = 0.0
        self.data_out.y = 0.0
        self.data_out.theta = 0.0
        rospy.Subscriber("/wr", Float32, self.callbackWr)
        rospy.Subscriber("/wl", Float32, self.callbackWl)
        rospy.Subscriber("/goal", goal, self.callbackGoal)
        self.start_time = rospy.get_time()
        print("Position node is running")
    
    def wrapTheta(self, angle):
        #Keep the input angle between values of -pi and pi
        if(angle > np.pi or angle <= -np.pi):
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle


    def PI_Angular(self):
        #PID controller for angular velocity
        self.integral_angular += self.error_theta * self.dt
        output = self.kp_angular * self.error_theta + self.ki_angular * self.integral_angular
        self.last_error_theta = self.error_theta
        if(output > 0 and output < 0.05): output = 0.05
        elif(output > -0.05 and output < 0): output = -0.05
        return output
    
    def PI_Linear(self):
        #PID controller for linear velocity
        self.integral_linear += self.error_d * self.dt
        output = self.kp_linear * self.error_d + self.ki_linear * self.integral_linear
        output = self.kp_linear * self.error_d
        self.last_error_d = self.error_d
        return output


    def callbackWr(self, msg):
        self.wr = msg.data
    
    def callbackWl(self, msg):
        self.wl = msg.data
    
    def callbackGoal(self,msg):
        self.target_x = msg.pos_x
        self.target_y = msg.pos_y


    def stop(self):
        #On exit stop the puzzlebot
        self.speed.angular.z = 0.0
        self.speed.linear.x = 0.0
        self.main_pub.publish(self.speed)
        print("Stopping")


    def run(self):
        


        self.dt = rospy.get_time() - self.start_time


        if(self.beginning == False):
            print("Initializing")
        else:
            if(self.hold == True):
                rospy.sleep(0.1)
                self.hold = False


            # Update robot's pose and error values
            self.theta_k = self.theta_k + (self.r*((self.wr - self.wl) /self.l)) * self.dt
            self.theta_k = self.wrapTheta(self.theta_k)
            self.x_k = self.x_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.cos(self.theta_k)
            self.y_k = self.y_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.sin(self.theta_k)


            self.error_theta = (np.arctan2(self.target_y - self.y_k, self.target_x - self.x_k)) - self.theta_k
            self.error_d = np.sqrt((self.target_x-self.x_k)**2 + (self.target_y-self.y_k)**2)
            self.error_theta = self.wrapTheta(self.error_theta)
            self.data_out.x = self.x_k
            self.data_out.y = self.y_k
            self.data_out.theta = self.theta_k


           
             # Control logic for robot movement
        
            # If the angular error is greater than a threshold and the angular threshold is not reached yet
            if(np.rad2deg(abs(self.error_theta)) > 0.3 and self.angular_threshold_reached == False):
                self.speed.linear.x = 0.0
                self.speed.angular.z = self.PI_Angular()
                print("I'm moving angularly")
            
            # If the angular error is less than a threshold and the pause counter is less than 50 stop the robot
            elif(np.rad2deg(abs(self.error_theta)) < 0.3 and self.pause < 50):
                self.pause += 1
                self.angular_threshold_reached = True
                self.speed.linear.x = 0.
                self.speed.angular.z = 0.


            # If the linear error is greater than a threshold and the linear threshold is not reached yet
            elif(self.error_d > 0.1 and self.linear_threshold_reached == False):
                self.speed.linear.x = self.PI_Linear()
                self.speed.angular.z = 0.
                self.angular_threshold_reached = True
                print("I'm moving linearly")
                if(self.error_d > 4):
                    self.linear_threshold_reached = True


            # If none of the above conditions are met, stop the robot and reset relevant variables   
            else:
                self.speed.linear.x = 0.
                self.speed.angular.z = 0.
                if(self.target_x != self.last_target[0] or self.target_y != self.last_target[1]):
                    self.linear_threshold_reached = False
                    self.angular_threshold_reached = False
                    self.hold = True
                self.pause = 0
                self.integral_angular = 0
                self.last_target = [self.target_x, self.target_y]


             # Publish robot's speed, pose, and error values
            self.main_pub.publish(self.speed)
            self.pub.publish(self.data_out)
            self.pub2.publish(np.rad2deg(self.error_theta))
            self.pub3.publish(self.error_d)
            self.start_time = rospy.get_time()
        if(self.check > 2 ):
            self.beginning = True
        self.check += 1



if __name__ == '__main__':
    rospy.init_node("position_estimation")
    
    robot = puzzlebot()
    rate = rospy.Rate(100)
    rospy.on_shutdown(robot.stop)


    try:
        while not rospy.is_shutdown():
            robot.run()
            rate.sleep()
    except rospy.ROSInterruptException():
        pass

`````


## Resultados  

**<p align="center"> Videos de demostración en el robot</p>**

https://youtube.com/shorts/GYwQgxYg7TQ?feature=share


**<p align="center"> Videos de demostración del robot en el simulador Gazebo</p>**


En estos videos se puede ver como el robot realiza la trayectoria tanto en físico y en el simulador.

## Conclusiones

Para la solución del reto se necesitó saber en todo momento la posición del robot, se implemento un controlador PID por su simplicidad y eficacia en este tipo de sistema, sin embargo, requirió ajustes en las constantes de cada acción del controlador para funcionar correctamente en esta aplicación. Con el generador de trayectorias nos dimos cuenta que se puede automatizar los procesos en el robot y que será de gran ayuda en el reto final.
