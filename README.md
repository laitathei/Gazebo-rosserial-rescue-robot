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
1. Use Arduino IDE to upload control_WithEncode_new.ino to Arduino Mega: This file will provide the encoder value and allow user to change PWM via keyboard
2. roslaunch virtualrobotv2 gazebo.launch: This file will simulate the game_field and the robot
3. rosrun rosserial_python serial_node.py /dev/ttyACM0: This file will connect the real world robot and allow them synchronized
4. rosrun virtualrobotv2 teleop_twist_keyboard.py: This file will control the real world robot movement such as forward and backward
5. rosrun virtualrobotv2 Encoder_to_odom_final_version.py: This file will translate the encoder value to odometry message
6. rosrun virtualrobotv2 Encoder_to_odom_back_to_origin.py: This file allow to put the simulated car into the original position

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

### 2.1.3 teleop_twist_keyboard.py
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_teleop.png)
* Change those variable value will cause the real world robot PWM change larger or smaller
* Press W or X will cause the real world robot move faster or slower for forward and backward motion
* Press E or C will affect the real world robot turning right speed or tuning left speed respectively
```XML
moveBindings = {
        'i':(-1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }
```
* Above program means that the real world robot motion is controlled by Keyboard button
* Change the letter inside`'?'` can change the control button position
* Change the variable inside `(0,0,0,0)` can change the moving direction
* First column means the forward and backward motion
* Last column means the turn left and turn right motion
### 2.1.4 Encoder_to_odom_final_version.py
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_encoder_to_odom.png)
```XML
self.encoder_one_rotation = 1854.0
self.wheel_length = 19.0 # in cm
self.compensated_parameter = 0.23
```
* `encoder_one_rotation` is the approxmation of encoder value when the wheel rotate one cycle
* `wheel_length` means the real world world robot wheen length
* `The compensated_parameter` means the value that required in turning process to ensure the simulated robot synchronized with the real world robot
* If you found that the simulated robot slower than real world robot motion, you should increase compensated_parameter vice versa 
* These parameter can modified according to your situation which means every one should have their own values
* The program will receive the required information by subscribing two encoder topic and simulated world odometry topic
* When we increase the motor PWM, the program will tranform the encoder value to the odometry message and publish the odometry message to the simulated world odometry topic
* Therefore, the simulated robot will always changing its position to reach the final position instead of driving the wheel

### 2.1.5 Encoder_to_odom_back_to_origin.py
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_back_to_origin.png)
```XML
Please choose your robot motion
Please place the cursor to this window before refresh the car position!
1. Back to the origin facing forward
2. Back to the origin facing backward
3. Back to the origin facing to left
4. Back to the origin facing to right
5. Back to the origin facing quadrant 1
6. Back to the origin facing quadrant 2
7. Back to the origin facing quadrant 3
8. Back to the origin facing quadrant 4
9. Exit
Enter a choice:
```
* The objective of this program is reducing the time of reopen gazebo.launch file when the simulated robot finished its mission or make some mistake while moving the robot
* When typing 1 to 8 into the command line, the simulated robot will back to the original position with different orientation
* When typing 9 into the command line, it will leave the program

### 2.2 Power supply setup
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/demo_power_supply_setup.jpeg)

### 2.3 ROS graph
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/rqt_graph.png)

### 2.4 Logical graph
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/logical_graph.png)

### 3 Encountered Problem solution
### 3.1.1 ROS problem 1
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/ROS_problem1.png)
* First, Please make sure that you have put the file in correct directory or it just a typo mistake
* Second, type `source ~/catkin_ws/devel/setup.bash` in command line
* Third, type `catkin_make` in ~/catkin_ws directory

### 3.1.2 ROS problem 2
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/ROS_problem2.png)
* First, open a new command line and type `roscore`
* Second, open a new command line again and type `roslaunch virtualrobotv2 gazebo.launch`
* Third, open these files again and the problem should be solved

### 3.1.3 Arduino problem
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/Arduino_problem.png)
* First, check whether you have turn on any ROS program such as rosrun rosserial_python serial_node.py /dev/ttyACM0
* Second, upload Arduino program again while closed all the ROS program
* Remeber! ROS and Arduino cannot run both at the same time. Please make sure that turn off another when you want to upload or open program

### 4 Gazebo Web
### 4.1 Installation
Type the following code into command line to install Gazebo Web
```
sudo apt install gazebo9 libgazebo9-dev
sudo apt install libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
source ~/.bashrc
nvm install 8
cd ~; git clone https://github.com/osrf/gzweb
cd ~/gzweb
git checkout gzweb_1.4.1
source /usr/share/gazebo/setup.sh
npm run deploy --- -m local # only load the local model
npm update minimatch@3.0.2
npm update -d
npm i ajv
npm install ajv@^5.0.0
npm audit fix
npm audit fix --force
npm run deploy --- -m local
```
### 4.2 Turn on Gazebo Web (synchronized with Gazebo)
Remember to open your gazebo world first
```
cd ~/gzweb/
npm start
```
Open thr browser and type ```https://your_current_ip_address:8080```

### 5 Network setting
### 5.1 Network connection
* The master device and the slave device will connect with local lan to have high speed data transmission rate
* As Web service included in this subject, both master and slave device required to have their own IP address
* Router will be used for connect the internet and other device will connect to router with DHCP policy
* Remember to configure the router setting first
* Base on DHCP policy, all the connected device got their own IP address
* These IP address can implement in ROS master URI (Please view the README:https://github.com/laitathei/Gazebo-rosserial-rescue-robot/tree/main/yolov4-tiny)
