# Gazebo-rosserial-rescue-robot

### Required command for keyboard control in both world
```XML
roslaunch virtualrobot gazebo.launch
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun virtualrobot control_WithEnodeValue.py
```
