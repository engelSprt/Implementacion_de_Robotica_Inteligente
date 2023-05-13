#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import Pose2D
from mini_challenge_3.msg import errores

class controller:
    def __init__(self):
        
        # Variables para el almacenamiento de las velocidades de las ruedas

        #Variables para recibir los puntos deseados
        self.target_x = 0.0
        self.target_y = 0.0
        
        self.tiempo_muestreo = 0.1

        self.r = 0.05
        self.l = 0.19

        self.first = True

        

        # Variables para el almacenamiento de las posiciones del robot
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        #Variables para alamcenar los tiempos
        self.current_time = 0.0
        self.previous_time = 0.0
        
        #self.target_x = 0.0
        #self.target_y = 0.0 
        #self.theta_k = 0.0


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

        self.pwm = 0


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


        #self.i = 0

        #Inicializar nodos
        rospy.init_node("controller")
        #rospy.Subscriber("/wr", Float32, self.wr_callback)
        #rospy.Subscriber("/wl", Float32, self.wl_callback)
        #rospy.Subscriber("/camino", camino, self.camino_callback)
        rospy.Subscriber("/traffic_light", String, self.vision_callback)   
        
        rospy.Subscriber("/Err", errores, self.callback_error)
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.semaforo_estado = rospy.Publisher('/cmd_vel', UInt8, queue_size = 10)

        self.rate = rospy.Rate(20)

        # Callbacks para velocidades de las ruedas
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

    def vision_callback(self,data):
        if data.data == "Stop":
            self.pwm = 0
        elif data.data == "Slow down":
            self.pwm = 120
        elif data.data == "Go":
            self.pwm = 255
        else:
            return
    
    #def camino_callback(self,msg):
    #    self.target_x = msg.camino_X
    #    self.target_y= msg.camino_y

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

                self.semaforo_estado.publish(self.pwm)
                #if dt >= self.tiempo_muestreo:
                # Actualizar las posiciones
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
                ##################################################################################
                if self.error1 == 0 or self.error2 == 0:
                #Publicar las posiciones
                    #self.err_pub.publish(self.el_errores)
                    self.speed.linear.x = 0
                    self.speed.angular.z = 0

                self.speed.linear.x = self.respuesta_linear
                self.speed.angular.z = self.respuesta_angular
                #print("Si estoy publicando las velocidades controladas \n")
                self.vel_pub.publish(self.speed)
                print("Si estoy publicando las velocidades controladas \n")
                self.rate.sleep()
if __name__ == "__main__":
    Controller = controller()
    try:
        Controller.run()
    except rospy.ROSInterruptException:
        None 
