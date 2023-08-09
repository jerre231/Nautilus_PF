#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
from geometry_msgs.msg import PointStamped, Twist
import math

"""def callbackCamera(data):
    rospy.loginfo("Received camera orientation data")
    rate = rospy.Rate(10)  # 10 Hz
    global """

def callback(data):
    rospy.loginfo("Received sonar data x=%f y=%f", data.point.x, data.point.y)
    global sonar_data
    sonar_data = data

def follow_nemo():
    global sonar_data
    rospy.init_node('following', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/sonar_data', PointStamped, callback)
    target_distance = 1.0 # distancia desejada em relação ao nemo

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if sonar_data:
            target_distance_x = sonar_data.point.x
            target_distance_y = sonar_data.point.y
            module_distance = math.sqrt(math.pow(target_distance_x, 2) + math.pow(target_distance_y, 2))

            comando = Twist()
            if (module_distance >= target_distance):
                comando.linear.y = 0.2
                comando.angular.z = -0.1 * target_distance_y
            else:
                comando.linear.y = 0

            pub.publish(comando)
        rate.sleep()

if __name__ == "__main__":
    sonar_data = None
    follow_nemo()
    rospy.spin()
