#!/usr/bin/env python3

import rospy
import numpy as np
import os

import rospy, rospkg, cv2, cv_bridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MoveDumbbells(object):

    def __init__(self):

        self.initalized = False

        rospy.init_node('move_dumbbells')

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg = Twist()

        self.proportional_control = 0.005

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # self.q_matrix = np.loadtxt("output_matrix.txt")

        self.goal_db = "red"
        self.goal_box = "one"

        # set color boundaries
        self.lower_red = np.array([110, 100, 100])
        self.upper_red = np.array([130, 255, 255])

        # Set up proportional control variables
        self.x_mid = 0
        self.constant_x = 0.05

        self.initialized = True

    def laser_callback(self, data):
        # If goals aren't set yet, do nothing
        if not self.initalized:
            return

        # Check if close to goal object, if so stop otherwise keep moving

    def image_callback(self, data):
        # If goals aren't set yet, do nothing
        if not self.initalized:
            return

        # take the ROS message with the image and turn it into a format cv2 can use
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if self.goal_db == 'red':
            mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        
        # erases all pixels that aren't specified color
        h, w, d = img.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        self.x_mid = int(w / 2)

        # using moments() function, the center of the colored pixels is determined
        M = cv2.moments(mask)

        # if there are any colored pixels found
        if M['m00'] > 0:
                # center of the yellow pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # a black circle is visualized in the debugging window to indicate
                # the center point of the colored pixels
                cv2.circle(img, (cx, cy), 20, (255,255,255), -1)

                error_x = self.x_mid - cx
                error_y = cy - self.y_bot

                proportional_control_x = self.constant_x * error_x
                proportional_control_y = self.constant_y * error_y

                self.msg.linear.x = 0.25
                self.msg.angular.z = proportional_control_x / 10

                print("Velocity:", self.msg.linear.x, "Angular:", self.msg.angular.z)

                self.publisher.publish(self.msg)

        # shows the debugging window
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
        node = MoveDumbbells()
        node.run()