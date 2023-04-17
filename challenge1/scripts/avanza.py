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
