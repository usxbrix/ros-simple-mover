# simple_movement

Various ROS nodes to move a robot or measure speed/velocity.

_WARNING!!!_

_Ugly code included! I only create quick and dirty code for testing purposes and prototypes. I would prefere if someone else would implement this in a better way._

based on the turtelsim tutorials from wiki.ros.org:
http://wiki.ros.org/turtlesim/Tutorials#Practicing_Python_with_Turtlesim


## nodes

### range2speed2.py

range2speed2 uses range sensor (messages) to calculate speed/velocity.
* When the robot is moving to a target (e.g a wall) the measuring begins a soon as the target is within the range of the range sensor (currently 1.2m).
* The robot and the measuring stops when range is below 0.15m

#### RUN
`rosrun simple_movement range2speed2.py`

### range.py

robot moves forward and turns when reaching an obstactle (range below 0.20cm).
#### RUN
`rosrun simple_movement range.py`