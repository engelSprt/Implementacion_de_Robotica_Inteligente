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
