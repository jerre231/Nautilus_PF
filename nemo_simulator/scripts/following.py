#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import math

class FollowGreenNode:
    def __init__(self):
        rospy.init_node('follow_green_node', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.target_color = (0, 255, 0)  # Green color in BGR format
        self.target_area = 300  # Minimum area of green blob to follow
        self.target_distance = 1.0  # Desired distance from the green blob

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            print(e)
            return
        
        mask = self.create_mask(cv_image)
        target = self.detect_target(mask)

        if target is not None:
            self.follow_target(target)

    def create_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([35, 100, 100])
        upper_bound = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def detect_target(self, mask):
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            if cv2.contourArea(contour) > self.target_area:
                M = cv2.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                return (cX, cY)
        
        return None

    def follow_target(self, target):
        image_center_x = 320  # Assuming image width is 640 pixels
        angular_speed = -4 * (target[0] - image_center_x) / image_center_x
        linear_speed = 1.7
        
        cmd = Twist()
        cmd.linear.y = linear_speed
        cmd.angular.z = angular_speed
        self.pub.publish(cmd)

def main():
    follow_green_node = FollowGreenNode()
    rospy.spin()

if __name__ == "__main__":
    main()
