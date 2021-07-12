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

buffertime_forward_mode = 180000
buffertime_backward_mode = 180000
buffertime_left_mode = 70000
buffertime_right_mode = 70000
buffertime_clockwise_mode = 45
buffertime_anticlockwise_mode = 45
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
global init_orientation_anticlockwise
global moved_radian

if __name__ == "__main__":
    final_x_position = 0.0
    final_y_position = 0.0
    final_z_position = 0.0
    final_x_orientation = 0.0
    final_y_orientation = 0.0
    global final_z_orientation
    final_z_orientation = 0.0
    check = False
    rospy.init_node('back_to_origin', anonymous=True)
    while check == False:
        updateodom = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        print "Please choose your robot motion"
        print "Please place the cursor to this window before refresh the car position!"
        print "1. Back to the origin"
        print "2. Exit"

        choice = int(raw_input("Enter a choice: "))
        if choice == 1:
            data_odom = ModelState()
            data_odom.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom.pose.position.x = 0.0                  # unchange
            data_odom.pose.position.y = 0.0     # change
            data_odom.pose.position.z = 0.0               # unchange

            # orientation
            data_odom.pose.orientation.x = 0.0               # unchange
            data_odom.pose.orientation.y = 0.0               # unchange
            data_odom.pose.orientation.z = 0.0            # unchange
            data_odom.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom)
            final_x_position = data_odom.pose.position.x
            final_y_position = data_odom.pose.position.y
            final_z_position = data_odom.pose.position.z
            final_x_orientation = data_odom.pose.orientation.x
            final_y_orientation = data_odom.pose.orientation.y
            final_z_orientation = data_odom.pose.orientation.z
            final_w_orientation = data_odom.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(final_z_orientation)        # unchange
            print "Success!"
            check = False

        elif choice == 2:
            check = True
    while check == True:
        sys.exit()