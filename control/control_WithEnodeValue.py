#!/usr/bin/env python2.7

"""
This is a very simple control script to test the robot.
a : turn the robot to left  (angular z += basic left)
d : turn the robot to right (angular z += basic right)
w : add forward power
x : reset
"""
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from pynput import keyboard


basic_velocity_x = 0

x_step = -0.05
left_step = 0.05
right_step = -0.05
left_step_larger = 0.4
right_step_larger = -0.4
adjust = 0
power = basic_velocity_x


def ros_setup():
    #starts a new node
    rospy.init_node('robot', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    return vel_msg,velocity_publisher
    
def on_press (key):
    global adjust,power
    try:
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
    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    #print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def callbackL(data):
    print("encoderL:")
    print(data.data)                                #print the received message from rostopic

def callbackR(data):
    print("encoderR:")
    print(data.data)                                #print the received message from rostopic
        
    
   
    
def main():
    speed_msg,ros_publisher = ros_setup()
    print("ROS node initialization is done!")
    #This script supported keyboard control
    listener = keyboard.Listener(on_press=on_press,on_release=on_release)
    rospy.Subscriber("LeftEncoder_value", Int64, callbackL)    #subscribe the rostopic "LeftEncoder_value"
    rospy.Subscriber("RightEncoder_value", Int64, callbackR)   #subscribe the rostopic "RightEncoder_value"z
    listener.start()
    while not rospy.is_shutdown():
        speed_msg.linear.x = power
        speed_msg.linear.y = 0
        speed_msg.linear.z = 0
        speed_msg.angular.x = 0
        speed_msg.angular.y = 0
        global adjust
        speed_msg.angular.z = adjust
        print(speed_msg)
        ros_publisher.publish(speed_msg)
        time.sleep(0.05)

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
