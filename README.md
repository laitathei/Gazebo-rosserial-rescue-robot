# Gazebo-rosserial-rescue-robot

### 1.1 Software Requirement
1. Ubuntu 18.04
2. ROS melodic
3. Gazebo 9
4. Python 2.7

### 1.2 Hardware Requirement
1. Arduino Mega
2. L298N Motor Driver
3. JGB37-520B Motor

### 1.3 Startup Procedures
1. roslaunch virtualrobot gazebo.launch: This file will simulate the game_field and the robot
2. rosrun rosserial_python serial_node.py /dev/ttyACM0: This file will connect the real world robot and allow them synchronized
3. rosrun virtualrobot control_WithEnodeValue.py: This file will control the real world robot movement such as forward and backward
4. rosrun virtualrobotv2 Encoder_to_odom_with_negative.py: This file will translate the encoder value to odometry message
5. rosrun virtualrobotv2 Encoder_to_odom_back_to_origin.py: This file allow to put the simulated car into the original position

### 2.1 Program Explaination
### 2.1.1 Gazebo.launch
`<?xml version="1.0" encoding="UTF-8"?>`Specify that it is an XML file
`<launch>...</launch>`Specify that it is an launch file and all the code should be inside this tag
`<arg name = "xxx" default= "xxx"/>`Similiar with C++ to define variable name and the value
`<include file="path....."/>`Similiar with C++ to include header file or source file
