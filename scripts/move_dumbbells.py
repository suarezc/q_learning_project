#!/usr/bin/env python3

import rospy
import numpy as np
import os
import keras_ocr
import math

import rospy, rospkg, cv2, cv_bridge
import moveit_commander

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist, Vector3


path_prefix = os.path.dirname(__file__) + "/action_states/"
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
        self.twist_msg = Twist()

        self.proportional_control = 0.005

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # moving constructs
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.q_matrix = np.loadtxt("output_q_matrix.txt")

        self.colors = []
        self.blocks = []
        
        self.populate_goals()
        self.goal_db = self.colors.pop(0)
        self.goal_box = None

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

        # Set numerical "boundaries"
        self.acceptable_digits = {
            "one": ['1', 'l', 'i', 't'],
            "two": ['2'],
            'three': ['3', 's', 'S']
        }

        # Control booleans

        #stop in front of right dumbbell
        self.stop_moving = False

        #set to true when we find a block
        self.ignore_lidar = False

        #set to true when dumbbell is grabbed
        self.holding_db = False

        #
        self.block_detected = False

        # Set up proportional control variables
        self.x_mid = 0
        self.constant_x = 0.05

        self.move_group_arm.go([0,0,0,0], wait=True)        
        arm_joint_goal = [0.0, 0.85, -0.75, -0.15]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        gripper_joint_goal = [0.009, -0.009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        self.initialized = True

        print("inited")

    def populate_goals(self):
        colors = ['red', 'green', 'blue']
        blocks = ['one','two', 'three']
        max_val = 0
        state = 0
        while max_val != 100:
            max_val = max(self.q_matrix[state])
            action = self.q_matrix[state].tolist().index(max_val)
            state = self.action_matrix[state].tolist().index(action)

            self.colors.append(colors[int(self.actions[int(action)][0])])
            self.blocks.append(blocks[int(self.actions[int(action)][1]) -1])
        print("colors:", self.colors)
        print('blocks:', self.blocks)

    def laser_callback(self, data):
        # If goals aren't set yet, do nothing
        # print("laser_callback")
        # print("stop_moving:", self.stop_moving)
        # print("ignore_lidar:", self.ignore_lidar)
        # print('block_detected:', self.block_detected)
        # print('holding_db:', self.holding_db)

        if not self.initialized or self.stop_moving or self.ignore_lidar:
            return

        # Check if close to goal object
        closest = 3.5
        distance = 0.23 if not self.holding_db else 0.5
        for i in range(45):
            index = (338 + i) % 360
            closest = closest if closest < data.ranges[index] else data.ranges[index]
        if closest < distance:
            self.stop_moving = True

            self.twist_msg.linear.x = 0
            self.twist_msg.angular.z = 0
            self.twist_pub.publish(self.twist_msg)
            print("Stopping:", closest)

            # Pick up dumbbell
            if not self.holding_db:
                print("in holding_db false")
                # Tilt back hand once dumbbell is grabbed
                arm_joint_goal = [0.0, 0.85, -0.75, -0.3]
                self.move_group_arm.go(arm_joint_goal, wait=True)
                self.move_group_arm.stop()

                # Move dumbbell over center of robot
                arm_joint_goal = [0.0, 0.00, -0.50, -0.3]
                self.move_group_arm.go(arm_joint_goal, wait=True)
                self.move_group_arm.stop()

                self.goal_db = None
                self.goal_box = self.blocks.pop(0)
                self.ignore_lidar = True
                self.block_detected = False

                self.stop_moving = False
                self.holding_db = True
            # Put down dumbbell
            else:
                print("in holding_db true")
                # Reset arm to initial position
                arm_joint_goal = [0.0, 0.85, -0.75, -0.3]
                self.move_group_arm.go(arm_joint_goal, wait=True)
                self.move_group_arm.stop()

                # Move backwards for 1 second
                self.twist_msg.linear.x = -0.1
                self.twist_msg.angular.z = 0
                self.twist_pub.publish(self.twist_msg)
                rospy.sleep(3)

                self.twist_msg.linear.x = 0
                self.twist_msg.angular.z = 0
                self.twist_pub.publish(self.twist_msg)

                arm_joint_goal = [0.0, 0.85, -0.75, -0.15]
                self.move_group_arm.go(arm_joint_goal, wait=True)
                self.move_group_arm.stop()

                self.goal_db = self.colors.pop(0)
                self.goal_box = None
                self.ignore_lidar = True
                self.holding_db = False
                self.block_detected = False
                self.stop_moving = False

    def image_callback(self, data):
        # If goals aren't set yet, do nothing
        # print("image_callback")
        # print("stop_moving:", self.stop_moving)
        # print("ignore_lidar:", self.ignore_lidar)
        # print('block_detected:', self.block_detected)
        # print('holding_db:', self.holding_db)
        if (not self.initialized) or self.stop_moving:
            return

        

        # take the ROS message with the image and turn it into a format cv2 can use
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w, d = img.shape

        if self.goal_box:
            cv2.imshow("window", img)
            cv2.waitKey(3)
            if self.counter < 75:
                self.counter += 1
                return
            print("we here")
            
            # Reset counter and stop robot so it may process in peace
            self.counter = 0
            old_linear = self.twist_msg.linear.x
            old_angular = self.twist_msg.angular.z
            
            if not self.block_detected:
                self.twist_msg.linear.x = 0
                self.twist_msg.angular.z = 0
                self.twist_pub.publish(self.twist_msg)

            prediction_group = self.pipeline.recognize([img])
            if not prediction_group[0] or not prediction_group[0][0]:
                print('empty')
                self.twist_msg = Twist(
                    linear=Vector3(0, 0, 0),
                    angular=Vector3(0, 0, 0.25)
                )
                self.twist_pub.publish(self.twist_msg)
                return

            print(prediction_group[0][0])

            acceptable = self.acceptable_digits[self.goal_box]

            # If nothing detected, maintain former velocity
            if (prediction_group[0][0][0] not in acceptable):
                self.twist_msg.linear.x = old_linear
                self.twist_msg.angular.z = old_angular
                self.twist_pub.publish(self.twist_msg)
                return
        
            # Otherwise unblock proximity check and adjust angular velocity
            self.ignore_lidar = False
            self.block_detected = True
            cx = 0
            for x in prediction_group[0][0][1]:
                cx += x[0]
            cx = cx/4
            

            kp = 0.03
            #318, 369
            diff = (cx - w/2)/180
            angular = diff * kp
            self.twist_msg = Twist(
                linear=Vector3(0.1, 0, 0),
                angular=Vector3(0, 0, -angular)
                )
            self.twist_pub.publish(self.twist_msg)
            print(self.twist_msg)


            # cv2.imshow("window", img)
            # cv2.waitKey(3)
            return

        if not self.goal_db:
            print("No goal dumbbell")
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
        if M['m00'] > 0:
            self.ignore_lidar = False
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
            self.twist_msg = Twist(
                linear=Vector3(0.1, 0, 0),
                angular=Vector3(0, 0, -angular)
                )
            self.twist_pub.publish(self.twist_msg)

            #     # shows the debugging window
            cv2.imshow("window", img)
            cv2.waitKey(3)
        else:
            self.twist_msg = Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, 0.25)
                )
            self.twist_pub.publish(self.twist_msg)
    # cv2.imshow("window", img)
    # cv2.waitKey(3)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
        node = MoveDumbbells()
        node.run()