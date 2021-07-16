#!/usr/bin/env python2.7
import time
import rospy
import math
import sys
import geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs import msg
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int64, Header, String
from gazebo_msgs.msg import ModelState


current_left = 0
current_right = 0
encoder_one_rotation = 1854.0
wheel_length = 19.0       #15.7079632679 is come from my calculation but 16.0 is more suitable to simulation

global moved_distance
global moved_x_distance
global moved_angular
global right_wheel_moved
global left_wheel_moved
global chassis_radius
global distance_from_y_origin
global final_x_position
global final_y_position
global final_z_position
global pi
global init_orientation
global moved_radian

moved_radian = 0.0
pi = 3.14159265359
init_orientation = 0.707106060115
moved_distance = 0.0
moved_x_distance = 0.0
moved_angular_distance = 0.0
right_wheel_moved = 0.0
left_wheel_moved = 0.0
chassis_radius = 0.0
distance_from_y_origin = 0.0

def callbackL(data):
    global current_left
    current_left = int(data.data)                             #print the received message from rostopic

def callbackR(data):
    global current_right
    current_right = int(data.data)                              #print the received message from rostopic

def fake_callback(data):
    global current_x_position
    global current_y_position
    global current_z_position
    global current_x_orientation
    global current_y_orientation
    global current_z_orientation
    global current_z_orientation

    current_x_position = float(data.pose.pose.position.x)
    current_y_position = float(data.pose.pose.position.y)
    current_z_position = float(data.pose.pose.position.z)
    current_x_orientation = float(data.pose.pose.orientation.x)
    current_y_orientation = float(data.pose.pose.orientation.y)
    current_z_orientation = float(data.pose.pose.orientation.z)
    current_w_orientation = float(data.pose.pose.orientation.z)

if __name__ == "__main__":
    final_x_position = 0.0
    final_y_position = 0.0
    final_z_position = 0.0
    final_x_orientation = 0.0
    final_y_orientation = 0.0
    global distance_from_x_origin
    distance_from_x_origin = 0.0
    global final_z_orientation
    final_z_orientation = 0.0
    global saved_x_position
    global saved_x_position_negative
    global saved_y_position
    global saved_y_position_negative
    global saved_left
    global saved_right
    saved_left = 0.0
    saved_right = 0.0
    saved_x_position = 0.0
    saved_x_position_negative = 0.0
    saved_y_position = 0.0
    saved_y_position_negative = 0.0
    current_x_position = 0.0
    current_y_position = 0.0
    current_z_position = 0.0
    current_x_orientation = 0.0
    current_y_orientation = 0.0
    current_z_orientation = 0.0
    current_z_orientation = 0.0
    rospy.init_node('odometry', anonymous=True)
    while not rospy.is_shutdown():
        saved_x_position = distance_from_x_origin
        saved_x_position_negative = distance_from_x_origin
        saved_y_position = distance_from_y_origin
        saved_y_position_negative = distance_from_y_origin
        rospy.Subscriber("/LeftEncoder_value", Int64, callbackL)    #subscribe the rostopic "LeftEncoder_value"
        rospy.Subscriber("/RightEncoder_value", Int64, callbackR)   #subscribe the rostopic "RightEncoder_value"
        rospy.Subscriber("/ground_truth/state", Odometry, fake_callback)   #subscribe the rostopic "/ground_truth/state"
        updateodom = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        saved_left = current_left
        saved_right = current_right

        data_odom = ModelState()
        print "The current LeftEncoder_value is : "+ str(current_left)
        print "The current RightEncoder_value is : "+ str(current_right)
        print "The previous x-axis position is : "+ str(current_x_position*100.0)
        print "The previous y-axis position is : "+ str(current_y_position*100.0)
        print "The previous z-axis position is : "+ str(current_z_position)
        print "The previous z-axis orientation is : "+ str(final_z_orientation)
        print "The previous LeftEncoder_value is : "+ str(saved_left)
        print "The previous RightEncoder_value is : "+ str(saved_right)
        print "The saved x-axis distance is : "+ str(saved_x_position)
        print "The saved y-axis distance is : "+ str(saved_y_position)

        # calculate the distance
        left_wheel_diff = float(current_left-saved_left)
        right_wheel_diff = float(current_right-saved_right)
        left_wheel_moved = (left_wheel_diff/encoder_one_rotation)*wheel_length
        right_wheel_moved = (right_wheel_diff/encoder_one_rotation)*wheel_length
        moved_distance =  (right_wheel_moved+left_wheel_moved)/2.0
        print "The moved distance (cm) is : "+ str(moved_distance)
                    

        # prepare the required information for set_model_state topic
        # Header
        data_odom.model_name = 'robot'
        #data_odom.header.frame_id = "world"
        # from geometry_msgs.msg.TransformStamped()
        #data_frame.child_frame_id = "base_footprint"

        # pose
        data_odom.pose.position.x = current_x_position*100.0                  # change
        data_odom.pose.position.y = saved_y_position/100.0     # unchange
        data_odom.pose.position.z = 0.0               # unchange

        # orientation
        data_odom.pose.orientation.x = 0.0               # unchange
        data_odom.pose.orientation.y = 0.0               # unchange
        data_odom.pose.orientation.z = final_z_orientation            # change
        data_odom.pose.orientation.w = 1.0            # unchange

        # update the setmodelstate 
        updateodom.publish(data_odom)
        distance_from_x_origin = moved_distance+saved_x_position
        final_z_position = data_odom.pose.position.z

        print "The final x-axis position (cm) is : "+ str(distance_from_x_origin)        # change
        print "The final y-axis position (cm) is : "+ str(saved_y_position)        # unchange
        print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
        print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
        print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
        print "The final z-axis orientation is : "+ str(final_z_orientation)        # unchange

    rospy.sleep(rospy.Duration(1.0))