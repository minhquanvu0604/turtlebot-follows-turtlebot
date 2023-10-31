# turtlebot-follows-turtlebot
A project for 41014 Sensor and Control for Mechatronic Systems

To launch the environment with 2 turtlebots:

`roslaunch turtlebot_follows_turtlebot_gazebo main.launch`

To run the Aruco detection and controller node:

`rosrun turtlebot_follows_turtlebot_navigation turtlebot_follows_turtlebot_navigation_aruco_detect`


Contribution:

Minh Quan Vu - Build simulated world; Implemented ArUco pose estimation; Implemented controller for follower TurtleBots. Contributed 60%

Anh Minh Tu - Hardware installation; Setup ROS and TurtleBots with RealSense usage; Implemented code into actual TurtleBots for the task. Contributed 40%
