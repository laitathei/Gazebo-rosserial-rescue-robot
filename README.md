# Gazebo-rosserial-rescue-robot

### Required command for keyboard control in both world
```XML
roslaunch virtualrobot gazebo.launch
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun virtualrobot control_WithEnodeValue.py
rosrun virtualrobot Encoder_to_odom_main.py
```
### Description for those files
1. roslaunch virtualrobot gazebo.launch: This file will simulate the game_field and the robot
2. rosrun rosserial_python serial_node.py /dev/ttyACM0: This file will connect the real world robot and allow them synchronized
3. rosrun virtualrobot control_WithEnodeValue.py: This file will control the real world robot movement such as forward and backward
4. rosrun virtualrobot Encoder_to_odom_main.py: This file will translate the encoder value to odometry message via choosing the correct motion
