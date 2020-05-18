# Miniproject

## Overview
The task of this project is to create a package that, when launched, will navigate the turtlebot to 20 different goal positions when published to the topic /goals. Additionaly the turtlebot will be avoiding the obstacles.

## Getting Started
In order to do this project following should be ready:
A compatible version of ubuntu with ROS Kinetic/Melodic installed and Gazebo model.


## Approach
The turtleboot will follow the imaginary line to reach to the goal point and if it finds any obstacle in between it will follow the obstacle. It will not hit the obstacle and will keep the obstacle in the right by turning left. Once the obstacle is avoided it will again start navigating straight to the next goal following the imaginary line.  

Two states are involved: Go to point and Follow Obstacle

## Implementation
The program starts with importing the required libraries like Points, euler_from_quaternion, and so on. There are three source codes one contains the information about the global variables used and two clients are created which calls the service and the other two source codes are executed. One source code(point.py) is about reaching the goal and other is avoiding the obstacle(obstacle.py).The two callback functions are implemented to subscribe to the messages of the position of goals and the robot.

## References

[1] https://www.youtube.com/playlist?list=PLK0b4e05LnzY2I4sXWTOA4_82cMh6tL
[2] https://bitbucket.org/theconstructcore/two-wheeled-robot-motion-planning/src/master/scripts/


