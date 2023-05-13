#!/usr/bin/env python

'''
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from mini_challenge_2.msg import errores

class Controller:
    def __init__(self):
        # Configurar parametros del controlador
        self.kp_l = 0.1 
        self.ki_l = 0.00
        self.kd_l = 0.0
        self.kp_ang = 1.5
        self.ki_ang = 0.01
        self.kd_ang = 0.1


        self.setpoint = 0.0
        self.error_sum_l = 0.0
        self.error_prev_l = 0.0
        self.error_diff_l = 0.0
        self.error_sum_ang = 0.0
        self.error_prev_ang = 0.0
        self.error_diff_ang = 0.0
        self.error_dist=0.0
        self.error_ang=0.0
        self.first=True
        self.velocidad_l=0
        self.velocidad_ang=0
        self.controlador_vl=0.0
        self.controlador_va=0.0        
        
        #Inicializar nodos
        rospy.init_node("controller")
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        rospy.Subscriber("/Err",errores,self.callback)   
        self.pose_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        
        self.rate = rospy.Rate(20)

    # Callbacks para velocidades de las ruedas
    def callback(self, msg):
        self.error_dist=msg.error_distancia
        self.error_ang=msg.error_angular

    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data
    
    def run(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time() 
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time
                #Controlador lineal  
                self.error_sum_l += self.error_dist * dt
                self.error_diff_l = (self.error_dist - self.error_prev_l) / (dt+0.000000001)
                self.error_prev_l = self.error_dist
                self.controlador_vl= self.kp_l * self.error_dist + self.ki_l * self.error_sum_l + self.kd_l * self.error_diff_l
                self.error_sum_ang += self.error_ang * dt
                self.error_diff_ang = (self.error_ang - self.error_prev_ang) / (dt+0.00001)
                self.error_prev_ang = self.error_ang
                self.controlador_va =self.kp_ang * self.error_ang + self.ki_ang * self.error_sum_ang + self.kd_ang * self.error_diff_ang
                #Publicar las posiciones
                msg.linear.x = self.controlador_vl#self.controlador_vl
                msg.angular.z = self.controlador_va
                if self.error_ang==0 and self.error_dist==0:
                    msg.linear.x = 0
                    msg.angular.z = 0
                print("Controlador_on")
                print_info = "%3f | %3f" %(self.controlador_va,self.controlador_vl)
                rospy.loginfo(print_info)
                self.pose_pub.publish(msg)
                self.rate.sleep()
    def stop(self):
        print("Stopping")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.pose_pub.publish(self.msg)

if __name__ == "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None

'''


import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import Pose2D
from mini_challenge_2.msg import errores
from mini_challenge_2.msg import camino

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
        rospy.Subscriber("/Err", errores, self.callback_error)   
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.rate = rospy.Rate(20)

        # Callbacks para velocidades de las ruedas
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

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
 
             
'''
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import Pose2D
from mini_challenge_2.msg import errores
from mini_challenge_2.msg import camino

class controller:
    def __init__(self):
        
        # Variables para el almacenamiento de las velocidades de las ruedas

        #self.wr = 0.0
        #self.wl = 0.0

        #Variables para recibir los puntos deseados
        self.target_x = 0.0
        self.target_y = 0.0
        
        self.wr = 0.0
        self.wl = 0.0

        self.tiempo_muestreo = 0.1
        #self.array_x = []
        #self.array_y = []
        
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


        self.el_errores = errores()
        self.el_errores.error_distancia = 0.0
        self.el_errores.error_angular = 0.0

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


        # Initialize PID controller gains and integrals
        self.kp_angular = 0.5
        self.ki_angular = 0.0
        self.kd_angular = 0.0

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
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        rospy.Subscriber("/camino", camino, self.camino_callback)
        self.err_pub = rospy.Publisher("/Err", errores, queue_size = 10)   
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.rate = rospy.Rate(10)

        # Callbacks para velocidades de las ruedas
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

    def camino_callback(self,msg):
        self.target_x = msg.camino_X
        self.target_y= msg.camino_y
        
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
                
                if dt >= self.tiempo_muestreo:
                # Actualizar las posiciones
                    self.previous_time = self.current_time
                    self.pose.theta = self.wrapTheta(dt * self.r * ((self.wr - self.wl) / self.l))
                    self.pose.x += dt * self.r * ((self.wr + self.wl) / 2) * np.cos(self.pose.theta)
                    self.pose.y += dt * self.r * ((self.wr + self.wl) / 2) * np.sin(self.pose.theta)
                    #Con esta funcion modificamos los valores de pi para que esten dentro de un limite
                    #self.wrapTheta(self.pose.theta)
                    #
                    self.pose.x += dt * self.r * ((self.wr + self.wl) / 2) * np.cos(self.pose.theta)
                    self.pose.y += dt * self.r * ((self.wr + self.wl) / 2) * np.sin(self.pose.theta)                

                    self.superError_linear = 0.0
                    self.prevError_linear = 0.0

                    self.superError_angular = 0.0
                    self.prevError_angular = 0.0

                    self.el_errores.error_angular = self.wrapTheta(np.arctan2(self.target_y-self.pose.y, self.target_x-self.pose.x) - self.pose.theta)
                    self.el_errores.error_distancia = np.sqrt(np.square(self.target_x-self.pose.x) + np.square(self.target_y-self.pose.y))
                ##################################################################################
                
                #Controlador_lineal
                # P_linear
                    self.P_linear = self.kp_linear * self.el_errores.error_distancia

                # I_linear
                    self.I_linear = self.superError_linear * self.ki_linear

                # D_linear
                    self.D_linear = ((self.el_errores.error_distancia - self.prevError_linear)/dt)

                    self.prevError_linear = self.el_errores.error_distancia

                    self.respuesta_linear = self.P_linear + self.I_linear + self.kd_linear*self.D_linear #Corregir
                ##################################################################################

                #Controlador_angular
                # P_angular
                    self.P_angular = self.kp_angular * self.el_errores.error_angular

                # I_angular
                    self.I_angular = self.superError_angular * self.ki_angular

                # D_angular
                    self.D_angular = ((self.el_errores.error_angular - self.prevError_angular)/dt)

                    self.prevError_angular = self.el_errores.error_angular

                    self.respuesta_angular = self.P_angular + self.I_angular + self.kd_angular*self.D_angular                
                ##################################################################################

                #Publicar las posiciones y el error
                    self.err_pub.publish(self.el_errores)
                    self.speed.linear.x = self.respuesta_linear
                    self.speed.angular.z = self.respuesta_angular
                #print_info = "%3f | %3f  " %(self.velocidad_l,self.velocidad_ang)
                #rospy.loginfo(print_info)
                    print("Si estoy publicando las velocidades controladas \n")
                    self.vel_pub.publish(self.speed)
                    
                    self.rate.sleep()

if __name__ == "__main__":
    func_run = controller()
    try:
        func_run.run()
    except rospy.ROSInterruptException:
        None      

'''