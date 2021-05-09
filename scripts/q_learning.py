#!/usr/bin/env python3

import rospy
import numpy as np
import os

from std_msgs.msg import Header
from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QMatrixRow, QLearningReward


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")
        self.alpha = 1
        self.gamma = 0.8
        self.current_state = 0
        self.prev_state = -1
        self.action = -1
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.update_q_matrix)

        self.counter = 0
        self.sum = -1
        self.current_iteration = 0

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-8 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        self.q_matrix = np.zeros((64, 9))

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        init_reward = QLearningReward()
        init_reward.header = Header(stamp=rospy.Time.now())
        init_reward.reward = 0
        init_reward.iteration_num = 0

        rospy.sleep(1)
        self.update_q_matrix(init_reward)


    
    def update_q_matrix(self, data):

        #Q Learning algorithm from project page implemented here
        reward = data.reward

        reward_calc = reward + self.gamma * max(self.q_matrix[self.current_state]) - self.q_matrix[self.prev_state][self.action]

        self.q_matrix[self.prev_state][self.action] += self.alpha * reward_calc

        #Create matrix msg
        matrix_msg = QMatrix()
        matrix_msg.header = Header(stamp=rospy.Time.now())
        matrix_msg.q_matrix = []

        #transform rows into something more palatable for the message and publisher
        for row in self.q_matrix:
            #print(row)
            q_row = QMatrixRow()
            q_row.q_matrix_row = row.astype(int).tolist()
            matrix_msg.q_matrix.append(q_row)
        
        self.q_matrix_pub.publish(matrix_msg)

        # call train so next message can arrive
        if(max(self.action_matrix[self.current_state]) < 0):
            self.reset_state()
        self.train_q_matrix()

        print("Completed up to", data.iteration_num, "rounds")

        
        #If we pass 100 iterations where the sum of the matrix is the same we have converged.
        if data.iteration_num > self.current_iteration + 100:
            if self.sum == np.sum(self.q_matrix):
                print("Q-matrix converged")
                print(self.q_matrix)
                np.savetxt("output_q_matrix.txt", self.q_matrix)
                rospy.signal_shutdown("Q-matrix has converged.")
                return
            
            self.current_iteration = data.iteration_num
            self.sum = np.sum(self.q_matrix)

    #For resetting our states for the train_q_matrix
    def reset_state(self):
        self.prev_state = -1
        self.current_state = 0
        self.action = -1

    #Choose random state that is possible from current state, get the corresponding action and send it
    def train_q_matrix(self):
        #ideal state is [3,1,2] aka state 39
        #start state is [0,0,0] aka state 0
        self.prev_state = self.current_state
        index = self.current_state
        action = -1
        choice_state = -1

        #choose valid random state
        while action == -1:
            choice_state = np.random.randint(64)
            action = int(self.action_matrix[index][choice_state])

        #Send message with our desired action and update our state and action trackers
        action_msg = RobotMoveDBToBlock()
        action_msg.robot_db = self.actions[action]["dumbbell"]
        action_msg.block_id = int(self.actions[action]["block"])
        self.current_state = choice_state
        self.action = action
        self.robot_action_pub.publish(action_msg)

        return

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = QLearning()
    node.run()
