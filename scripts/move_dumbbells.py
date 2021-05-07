#!/usr/bin/env python3

import rospy
import numpy as np
import os
import keras_ocr

import rospy, rospkg, cv2, cv_bridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist, Vector3

class MoveDumbbells(object):

    def __init__(self):

        self.initialized = False
        self.pipeline = keras_ocr.pipeline.Pipeline()
        self.counter = 0

        rospy.init_node('move_dumbbells')

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback, queue_size=None)

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg = Twist()

        self.proportional_control = 0.005

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # self.q_matrix = np.loadtxt("output_matrix.txt")

        self.goal_db = "bluea"
        self.goal_box = "one"

        # set color boundaries
        #blue works
        self.lower_blue = np.array([110,50,50])
        self.upper_blue = np.array([130, 255, 255])

        #green
        self.lower_green = np.array([50,100,100])
        self.upper_green = np.array([70, 255, 255])

        #red may need multiple masks but this works
        self.lower_red = np.array([0,100,100])
        self.upper_red = np.array([10, 255, 255])


        # Set up proportional control variables
        self.x_mid = 0
        self.constant_x = 0.05

        self.initialized = True
        print("inited")

    def laser_callback(self, data):
        # If goals aren't set yet, do nothing

        if not self.initialized:
            return

        # Check if close to goal object, if so stop otherwise keep moving

    def image_callback(self, data):
        # If goals aren't set yet, do nothing
        if not self.initialized:
            return

        print("we here")

        # take the ROS message with the image and turn it into a format cv2 can use
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w, d = img.shape

        if self.goal_box == 'one':

            if self.counter < 100:
                self.counter += 1
                return

            self.counter = 0
            prediction_group = self.pipeline.recognize([img])
            if not prediction_group[0] or not prediction_group[0][0]:
                print('empty')
                my_twist = Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, 0.1)
                )
                self.twist_pub.publish(my_twist)
                return

            print(prediction_group[0][0][0])
            if(prediction_group[0][0][0] != '1' and prediction_group[0][0][0] != 'l' and prediction_group[0][0][0] != 'i'):
                return
        
            cx = 0
            for x in prediction_group[0][0][1]:
                cx += x[0]
            cx = cx/4
            

            kp = 0.05
            #318, 369
            diff = (cx - w/2)/180
            angular = diff * kp
            my_twist = Twist(
                linear=Vector3(0.1, 0, 0),
                angular=Vector3(0, 0, -angular)
                )
            self.twist_pub.publish(my_twist)
            print(my_twist)


            cv2.imshow("window", img)
            cv2.waitKey(3)
            return


        if self.goal_db == 'blue':
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        if self.goal_db == 'red':
            mask = cv2.inRange(hsv, self.lower_red, self.upper_red)

        if self.goal_db == 'green':
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # erases all pixels that aren't specified color

        self.x_mid = int(w / 2)

        # using moments() function, the center of the colored pixels is determined
        M = cv2.moments(mask)
        # if there are any yellow pixels found
        print(M)
        if M['m00'] > 0:
            print("color pixel found")
                # center of the yellow pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
                

                   # a red circle is visualized in the debugging window to indicate
                   # the center point of the yellow pixels
            cv2.circle(img, (cx, cy), 20, (255,255,255), -1)

                
            kp = 0.5
                  #318, 369
            diff = (cx - w/2)/180
            angular = diff * kp
            my_twist = Twist(
                linear=Vector3(0.1, 0, 0),
                angular=Vector3(0, 0, -angular)
                )
            self.twist_pub.publish(my_twist)

                # shows the debugging window
            cv2.imshow("window", img)
            cv2.waitKey(3)
        else:
            my_twist = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0.1)
            )
            self.twist_pub.publish(my_twist)
        cv2.imshow("window", img)
        cv2.waitKey(3)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
        node = MoveDumbbells()
        node.run()