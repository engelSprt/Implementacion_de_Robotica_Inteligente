#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

class Controller:
    def __init__(self):
        self.tiempo_muestreo = 0.1
        self.r = 0.05
        self.l = 0.19
        self.nose = 0.0

        self.error_angular = 0.0
        self.respuesta_angular = 0.0

        # Initialize Twist message for robot speed
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

        # Initialize Pose2D message for robot pose
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        # Initialize PID controller gains
        self.kp_angular = 0.1
        self.ki_angular = 0.0
        self.kd_angular = 0.012
        
        # Initialize error and integral term for PID
        self.error_integral = 0.0
        self.prev_error = 0.0

        # Initialize ROS node and publishers/subscribers
        rospy.init_node("controller")
        rospy.Subscriber("ang_err", Float32, self.error_callback)
        rospy.Subscriber("nose", Float32, self.nose_callback)
        self.pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.prueba = rospy.Publisher('/velocidad_controlada', Twist, queue_size=10)
        self.rate = rospy.Rate(20)

    def error_callback(self, msg):
        self.error_angular = msg.data

    def nose_callback(self, msg):
        self.nose = msg.data

    def wrapTheta(self, theta):
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

    def stop(self):
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

    def run(self):
        while not rospy.is_shutdown():
            self.pose.theta = self.wrapTheta(self.pose.theta + self.tiempo_muestreo * self.r * self.respuesta_angular)
            self.pose.x += self.tiempo_muestreo * self.r * (self.speed.linear.x * np.cos(self.pose.theta))
            self.pose.y += self.tiempo_muestreo * self.r * (self.speed.linear.x * np.sin(self.pose.theta))
            
            self.pose_pub.publish(self.pose)

            rospy.loginfo("Angulo_recibido: %f", self.error_angular)

            # PID controller for angular error
            error = self.error_angular
            error_derivative = (error - self.prev_error) / (self.tiempo_muestreo)
            self.error_integral += error * self.tiempo_muestreo

            self.respuesta_angular = (self.kp_angular * error) + (self.ki_angular * self.error_integral) + (self.kd_angular * error_derivative)

            self.prev_error = error


            #print("Valor de self.nose:", self.nose)
            #rospy.loginfo("Valor recibido: %.2f", self.nose)

            self.speed.linear.x = 0.1
            self.speed.angular.z = (self.respuesta_angular/30)
            self.vel_pub.publish(self.speed)
            self.prueba.publish(self.speed)

            rospy.loginfo("Velocidad Angular: %f", self.speed.angular.z)

            self.rate.sleep()

if __name__ == "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass


'''
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
#from retosemaforo.msg import errores # modificar el paquete modificar el custom message

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

        self.wl = 0.0
        self.wr = 0.0

        
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
        self.kp_angular = 1.0
        self.ki_angular = 1.0
        self.kd_angular = 1.0

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
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        #rospy.Subscriber("/camino", camino, self.camino_callback)
       # rospy.Subscriber("/traffic_light", String, self.vision_callback)   
        
        rospy.Subscriber("/ang_err", Float32, self.callback_error)

        self.pose_pub = rospy.Publisher("/pose",Pose2D,queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        #self.semaforo_estado = rospy.Publisher('/cmd_vel', UInt8, queue_size = 10)

        self.rate = rospy.Rate(20)

        # Callbacks para velocidades de las ruedas
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

   # def vision_callback(self,data):
   #     if data.data == "Stop":
   #         self.estado = "Rojo"
   #     elif data.data == "Slow_down":
   #         self.estado = "Amarillo"
   #     elif data.data == "Go":
   #         self.estado = "Verde"
   #     else:
   #         return
    
    def callback_error(self,msg):
        self.error1 = msg.data

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
                self.P_angular = self.kp_angular * self.error1

                # I_angular
                self.I_angular += self.error1 * dt

                # D_angular
                self.D_angular = ((self.error1 - self.prev_Error_angular)/(dt+0.0000001))

                self.prev_Error_angular = self.error1

                self.respuesta_angular = self.P_angular + self.I_angular * self.ki_angular + self.kd_angular * self.D_angular                
                
                ##########################self.current_time = rospy.get_time()########################################################
                
               # if self.error1 == 0:
                    #Publicar las posiciones
                    #self.err_pub.publish(self.el_errores)
              #      self.speed.linear.x = 0
              #      self.speed.angular.z = 0

            # Semaforo en ROJO -> Se detiene el robot.

             #   if self.estado == "Rojo":
             #       self.speed.linear.x = 0
             #       self.speed.angular.z = 0
                    #print("Si estoy publicando las velocidades controladas \n")
                    #self.vel_pub.publish(self.speed)                   

            # Semaforo en Verde -> Avanza el robot.

            #    elif self.estado == "Verde":

            #        self.speed.linear.x = 0.1
            #        self.speed.angular.z = self.respuesta_angular
                    #print("Si estoy publicando las velocidades controladas \n")
                
            #    elif self.estado == "Amarillo":
            #        self.speed.linear.x -= 0.1
            #        self.speed.angular.z -= 0.1
            #        if self.speed.linear.x <= 0 or self.speed.angular.z <= 0:
                    # si la velocidad es cero, detener el robot
            #            self.stop()
                
            #    else:
                self.pose.theta = self.pose.theta + dt * self.r * ((self.wr - self.wl) / self.l)
                self.pose.x += dt * self.r * ((self.wr + self.wl) / 2) * np.cos(self.pose.theta)
                self.pose.y += dt * self.r * ((self.wr + self.wl) / 2) * np.sin(self.pose.theta)

                self.pose_pub.publish(self.pose)
                self.speed.linear.x = 0.05
                self.speed.angular.z = self.respuesta_angular
                rospy.loginfo("velocidad: %f", self.speed.angular.z)
            #    self.speed.angular.z = self.respuesta_angular
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

'''
