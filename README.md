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


## Timeline
* Implementation of Q-learning and testing convergence (simulating perception as needed) by 05/1
* Implementation of robot perception by 05/04
* Implementation of robot movement by 05/08
* Bug fix as needed until due date
