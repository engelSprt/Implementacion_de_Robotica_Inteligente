#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

class Controller:
    def __init__(self):
        # Configuracion de parametros
        self.tiempo_muestreo = 0.1
        self.r = 0.05
        self.l = 0.19
        self.nose = 0.0

        # Variables de control
        self.error_angular = 0.0
        self.respuesta_angular = 0.0

        # Inicializacion de velocidad
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

        # Inicializacion de posicion
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        # Coeficientes de control PID
        self.kp_angular = 0.1
        self.ki_angular = 0.0
        self.kd_angular = 0.012

        # Variables auxiliares de control
        self.error_integral = 0.0
        self.prev_error = 0.0

        # Inicializacion de nodo de ROS y suscripciones/publicaciones
        rospy.init_node("controller")
        rospy.Subscriber("ang_err", Float32, self.error_callback)
        rospy.Subscriber("nose", Float32, self.nose_callback)
        self.pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.prueba = rospy.Publisher('/velocidad_controlada', Twist, queue_size=10)
        self.rate = rospy.Rate(20)

    def error_callback(self, msg):
        # Callback para recibir el error angular
        self.error_angular = msg.data

    def nose_callback(self, msg):
        # Callback para recibir el valor de "nose"
        self.nose = msg.data

    def wrapTheta(self, theta):
        # Funcion para envolver el angulo theta entre -pi y pi
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

    def stop(self):
        # Detener el movimiento
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

    def run(self):
        while not rospy.is_shutdown():
            # Actualizar la posicion del robot en base a la velocidad angular
            self.pose.theta = self.wrapTheta(self.pose.theta + self.tiempo_muestreo * self.r * self.respuesta_angular)
            self.pose.x += self.tiempo_muestreo * self.r * (self.speed.linear.x * np.cos(self.pose.theta))
            self.pose.y += self.tiempo_muestreo * self.r * (self.speed.linear.x * np.sin(self.pose.theta))

            # Publicar la posicion actual
            self.pose_pub.publish(self.pose)

            rospy.loginfo("Angulo_recibido: %f", self.error_angular)

            # Calcular el controlador PID para la velocidad angular
            error = self.error_angular
            error_derivative = (error - self.prev_error) / (self.tiempo_muestreo)
            self.error_integral += error * self.tiempo_muestreo
            self.respuesta_angular = (self.kp_angular * error) + (self.ki_angular * self.error_integral) + (self.kd_angular * error_derivative)

            self.prev_error = error

            # Configurar la velocidad lineal y angular del robot
            self.speed.linear.x = 0.1
            self.speed.angular.z = (self.respuesta_angular/30)

            # Publicar la velocidad del robot
            self.vel_pub.publish(self.speed)
            self.prueba.publish(self.speed)

            rospy.loginfo("Velocidad Angular: %f", self.speed.angular.z)

            # Esperar hasta el siguiente ciclo de control
            self.rate.sleep()

if __name__ == "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
