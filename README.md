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
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_game_field.png)
* `<?xml version="1.0" encoding="UTF-8"?>`Specify that it is an XML file
* `<launch>...</launch>`Specify that it is an launch file and all the code should be inside this tag
* `<arg name = "xxx" default= "xxx"/>`Similiar with C++ to define variable name and the value
* `<include file="path..."/>`Similiar to include source file in C++ but this is special for include another XML launch file
* `<param name = "xxx".../>`Define the parameter name and what kind of parameter will be stored inside
* `<node name = "xxx".../>`Node can import ROS package and use it in the launch file

### 2.1.2 serial_node.py
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_rosserial.png)
* Allow the microcontroller to communicate with other electronic devices
* Every port will allocate with special port number
* While type the correct port name /xxx/xxxxx, the program connect the device

### 2.1.3 control_WithEnodeValue.py
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_control_WithEnodeValue.png)
```XML
x_step = -0.05
left_step_larger = 0.4
right_step_larger = -0.4
```
* Change those variable value will cause the real world robot PWM change larger or smaller
* Change the x_step value will cause the real world robot move faster or slower for forward and backward motion
* Change the left_step_larger or right_step_larger will affect the real world robot turning around speed respectively
```XML
        if(key.char=='a'):
            #Turn Left
            power = 0
            adjust += left_step_larger
        if(key.char=='d'):
            #Turn Right
            power = 0
            adjust += right_step_larger
        if(key.char=='w'):
            #Forward (increase the speed)
            power += x_step
            adjust = 0
        if(key.char=='s'):
            #Forward (increase the speed)
            power -= x_step
            adjust = 0
        if(key.char=='x'):
            #Stop
            power = 0
            adjust = 0    
```
* Above program means that the real world robot motion is controlled by WASDX button in Keyboard
* Change the letter inside`if(key.char=='?')` can change the control button position

### 2.1.4 Encoder_to_odom_with_negative.py
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_Encoder_to_odom.png)
```XML
self.encoder_one_rotation = 1854.0
self.wheel_length = 19.0 # in cm
self.compensated_parameter = 0.23
```
* `encoder_one_rotation` is the approxmation of encoder value when the wheel rotate one cycle
* `wheel_length` means the real world world robot wheen length
* `The compensated_parameter` means the value that required in turning process to ensure the simulated robot synchronized with the real world robot
* If you found that the simulated robot slower than real world robot motion, you should increase compensated_parameter vice versa 
* These parameter can modified according to your situation which means every one should have their own value
* The program will receive the required information by subscribing two encoder topic and simulated world odometry topic
* When we increase the motor PWM, the program will tranform the encoder value to the odometry message and publish the odometry message to the simulated world odometry topic
* Therefore, the simulated robot will always changing its position to reach the final position instead of driving the wheel

### 2.1.5 Encoder_to_odom_back_to_origin.py
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_back_to_origin.png)
```XML
Please choose your robot motion
Please place the cursor to this window before refresh the car position!
1. Back to the origin
2. Exit
Enter a choice: 
```
* The objective of this program is reducing the time of reopen gazebo.launch file when the simulated robot finished its mission or make some mistake while moving the robot
* When typing 1 into the command line, the simulated robot will back to the original position
* When typing 2 into the command line, it will leave the program

### 2.2 Power supply setup
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_power_supply_setup.jpeg)

### 3 Encountered Problem solution
### 3.1.1 ROS problem
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/ROS_problem1.png)
* First, Please make sure that you have put the file in correct directory or it just a typo mistake
* Second, type `source ~/catkin_ws/devel/setup.bash` in command line
* Third, type `catkin_make` in ~/catkin_ws directory
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/ROS_problem2.png)
* First, open a new command line and type `roscore`
* Second, open a new command line again and type `roslaunch virtualrobotv2 gazebo.launch`
* Third, open these files again and the problem should be solved
![image]()


