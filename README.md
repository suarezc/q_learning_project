# q_learning_project

## Implementation Plan

* Joseph Henry, Charles Suarez
* Q-learning Algorithm
  * We intend to implement an algorithm very similar to what we saw in class, modifications needed to produce functional codenonwithstanding. We believe the material covered in lecture should suffice in building a function we can iterate upon and better develop as needed. We will test by running it on small environments withfew actions see if we get qualities that we expect for the states from our experience in class.
  * We will know when our Q-matrix has converged when the output values stop changing or if they change in very non-consequential ways over several iterations. We can test this simply by running the Q-matrix and seeing if we get any stability or convergence in the values, if they are unstable after several hundred iterations we may need to revisit or revise as needed.
  * If our understanding is correct, the robot should simply execute the highest quality action it has available based on it's current state. This will likely involve subscribing and publishing to topics across multiple nodes. We can test if it is doing something useful by running our necessary nodes and seeing if the robot performs the expected behavior, if not we can debug the Q-matrix or our action selection as needed.
* Robot Perception
  * We can use the camera on the robot combined with our knowledge of the colors of the dumbells to identify which dumbell is which, we may peruse code from our follow the line class session for inspiration on how to use the camera. We will test this by having the robot turn towards a specific dumbell, if it chooses the right one we know we have the means to locate that dumbell.
  * We will use the computer vision library recommended (keras_ocr) along with the robots camera to scan the faces of the block and with a lidar scan we should be able to locate them in space and use that knowledge to navigate properly. Again to test we can have the robot face a block of a specific number that we want and check if it can find it. 
* Robot manipulation & movement
  * Using our knowledge from lecture and a generous amount of experimentation we should be able to find the joint angles and grip that give us a nice hold on the dumbell to move it. We can test by trying to lift a dumbell and see if it actually happens.
  * We will navigate our robot as follows. Once we pick up a dumbell of a certain color, our robot will rotate until we see the corresponding block in vision and then navigate towards it much like our wall_follower implementation we did long ago. We would use the proper topics for this and build the corresponding nodes as needed. To test we would have the robot grab a dumbell and navigate towards a block. Any block would do for testing movement, but if we have our recognition working we'll use the correct block. 


### Timeline
* Implementation of Q-learning and testing convergence (simulating perception as needed) by 05/1
* Implementation of robot perception by 05/04
* Implementation of robot movement by 05/08
* Bug fix as needed until due date

## Writeup
### Objectives Description
The goal of this project is to gain experience with Q-learning as well as sensory-motor components of robotics. Q-learning is used to determine the final location of three differently-colored dumbbells. Image processing is then used to identify the dumbbells and their goal locations, and a sensory arm to pick up and place the dumbbells.

### High-Level Description
Reinforcement learning was used to determine the correct location of the dumbbells. The robot uses Q-learning to gradually build up a matrix of advisable choices to make in any given scenario. During the training process, the robot is "rewarded" for reaching the goal state. Any states that led to that reward are also weighted slightly to create a gradient towards the goal. The result is a clear path from the origin state to the goal state.

### Q-Learning Algorithm Description
* Selecting and executing actions for the robot to take
  * This is accompished in `QLearning.train_q_matrix()`. Using the `action_matrix`, a random action is selected within the `current_state`th row. If this is an illegal action (-1), we continue selecting until a legal action is selected. This action is then used to create a `RobotMoveDBToBlock` message which is sent on the `/q_learning/robot_action` topic.

* Updating the Q-matrix
  * This is accomplished in `QLearning.update_q_matrix()`. Once the robot receives a reward it updates the Q-matrix using the algorithm from class. The new Q value is calculated and stored in `q_matrix[state][action]`. It then publishes this updated Q-matrix to the `/q_learning/q_matrix` topic.

* Determining when to stop iterating through the Q-learning algorithm
  * This is accomplished in `QLearning.update_q_matrix()`. We consider the Q-matrix converged after 1500 iterations. Once the world has been reset 1500 times, the node stops the program and prints out the resulting Q-matrix.

* Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot
  * This is accomplished in `MoveDumbbells.populate_goals()`. The node reads in the saved Q-matrix from the file as an array. It uses the Q-matrix to determine the most beneficial first action and then uses the action matrix to translate this into a color/number pairing. It then does this two more times to get a full set of instructions.

### Robot Perception Description
We did not use any outside sources for this and relied solely on the project specifications and the line follower and cat detection class exercises. 
* Identifying the locations and identities of each of the colored dumbbells
  * This is primarily accomplished in the `QLearning.image_callback()` function. We essentially use the same strategy as in line follower. We define a range of acceptable HSV values and convert them into OpenCV form. We use this mask to search for any pixels colored within that range.

* Identifying the locations and identities of each numbered blocks
  * This is primarily accomplished in the `QLearning.image_callback()` function. As recommended by the project specs, we use `keras_ocr` on incoming `Image` messages. Since this is quite computationally expensive, the robot only performs this when it is holding a dumbbell and even then only on every 75th `Image`. We also have to be a bit lenient and consider similar-looking alphanumeric characters as the same thing (e.g., consider an 'i' as a '1').

### Robot Manipulation and Movement
* Moving to the right spot in order to pick up a dumbbell
  * This is primarily accomplished in `QLearning.image_callback()`. We use a mask to search for any objects with the associated color and then use the center coordinates of the detected color blob to orient the robot via proportional control.

* Picking up the dumbbell
  * This is primarily accomplished in `QLearning.laser_callback()`. The robot lowers its arm forward when it is initialized. When it is close to the dumbbell (<0.23 meters), it will come to a stop. This is close enough for the arm to grab the dumbbell, and so it moves its arm up to pick up the dumbbell. The claw is initialized to be wider than the handle but slightly narrowed than the bell to allow for easy carrying.

* Moving to the desired destination (numbered block) with the dumbbell.
  * This is primarily accomplished in `QLearning.image_callback()`. We use `keras_ocr` to gain the x-coordinates of the identified digit within the robot's visual space. We then use proportional control to orient the robot toward the average of all returned x-coordinates. 

* Putting the dumbbell back down at the desired destination
  * This is primarily accomplished in `QLearning.laser_callback()`. Similar to above, the robot will stop when it is 0.5m away from the closest object in front of it (presumably the block). It then lowers its arm being mindful of the claw's position. After the arm is lowered, it slowly backs up to "release" the dumbbell. 

### Gif
![q_demonstration](/gif.gif)

  ### Challenges
  The most difficult challenge was gracefully picking up and releasing the dumbbell. The claw would usually be at an angle that caught some part of the dumbbell and knocked it over/away from the block. We overcame this with a lot of trial and error of different arm positions. We also tinkered with the amount of time spent backing up after placing a dumbbell. We have no "mathematical" evidence for why our final positions work the way they do, they are just the cumulation of our experience. Another difficult aspect of this assignment was figuring out how to maintain a coherent state. Since the robot has to respond to incoming data, your code can't really be perfectly linear (although it isn't really parallel either). So we had to use a set of four "control booleans" to keep track of our current state (e.g., holding dumbbell and moving toward block, looking for dumbbell, etc). 

  ### Future Work
  We would like to make our handling of dumbbells even more graceful. Although our current model achieves the goal, it often makes the dumbbells woble and occasionally it will knock one over. It would be really satisfying if our robot smoothly placed the dumbbells into their proper location and backed away without even brushing the object. Additionally, we would like to make our robot complete this task faster. Currently, our robot is extremely slow to maximize the chances of being perfectly aligned with the dumbbell and to maximize the number of times the image recognition software is run. However, it takes several minutes for the robot to move its three dumbbells and we feel we could do better than this given time.

  ### Takeaways
  * *It's good to spend a lot of time reviewing the starter code*: We spent nearly a full work-day just reviewing all of the starter code to make sure we knew exactly how everything worked. While this was very tedious and slow, I am certain it saved us a load of time on the long run. Once we started our implementation, things went really smoothly.
  * *Spend a lot of time on the implementation plan*: Similar to above, we spent a long while planning our our implementation. This was painful and agonizing, but we encountered relatively few hiccups while coding the assignment which was a huge plus. 
