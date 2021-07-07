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
global right_wheel_moved
global left_wheel_moved
global distance_from_origin
global final_x_position
global final_y_position
global final_z_position

moved_distance = 0.0
right_wheel_moved = 0.0
left_wheel_moved = 0.0
distance_from_origin = 0.0

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
    saved_left = 0
    saved_right = 0
    check = False
    rospy.init_node('odometry', anonymous=True)
    while check == False:
        rospy.Subscriber("/LeftEncoder_value", Int64, callbackL)    #subscribe the rostopic "LeftEncoder_value"
        rospy.Subscriber("/RightEncoder_value", Int64, callbackR)   #subscribe the rostopic "RightEncoder_value"
        rospy.Subscriber("/ground_truth/state", Odometry, fake_callback)   #subscribe the rostopic "RightEncoder_value"
        updateodom = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        saved_left = current_left
        saved_right = current_right

        print "Please choose your robot motion"
        print "Before control the car, please place the cursor to other window!"
        print "1. moving forward (press W two times immediately)"
        print "2. moving to left (press A two times immediately)"
        print "3. moving to right (press D two times immediately)"
        print "4. moving backward (press S two times immediately)"
        print "5. back to the origin"
        print "6. Exit"

        choice = int(raw_input("Enter a choice: "))
        
        if choice == 1:
            data_odom = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The current x-axis position is : "+ str(current_x_position)
            print "The current y-axis position is : "+ str(current_y_position)
            print "The current z-axis position is : "+ str(current_z_position)
            print "The previous LeftEncoder_value is : "+ str(saved_left)
            print "The previous RightEncoder_value is : "+ str(saved_right)

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
            data_odom.pose.position.x = 0.0                  # should be unchange
            data_odom.pose.position.y = moved_distance/100     # change
            data_odom.pose.position.z = 0.0               # unchange

            # orientation
            data_odom.pose.orientation.x = 0.0               # unchange
            data_odom.pose.orientation.y = 0.0               # unchange
            data_odom.pose.orientation.z = 0.0            # unchange
            data_odom.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom)
            distance_from_origin = moved_distance+current_y_position
            final_x_position = data_odom.pose.position.x
            final_z_position = data_odom.pose.position.z

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(distance_from_origin)        # change
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            check = False

        elif choice == 2:
            data_odom2 = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The current x-axis position is : "+ str(current_x_position)
            print "The current y-axis position is : "+ str(current_y_position)
            print "The current z-axis position is : "+ str(current_z_position)
            print "The previous LeftEncoder_value is : "+ str(saved_left)
            print "The previous RightEncoder_value is : "+ str(saved_right)

            # calculate the distance
            left_wheel_diff = float(current_left-saved_left)
            right_wheel_diff = float(current_right-saved_right)
            left_wheel_moved = (left_wheel_diff/encoder_one_rotation)*wheel_length
            right_wheel_moved = (right_wheel_diff/encoder_one_rotation)*wheel_length
            moved_distance =  (right_wheel_moved+left_wheel_moved)/2.0
            print "The moved distance (cm) is : "+ str(moved_distance)

            # prepare the required information for set_model_state topic
            # Header
            data_odom2.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom2.pose.position.x = 0.0                  # should be unchange
            data_odom2.pose.position.y = moved_distance/100     # change
            data_odom2.pose.position.z = 0.0               # unchange

            # orientation
            data_odom2.pose.orientation.x = 0.0               # unchange
            data_odom2.pose.orientation.y = 0.0               # unchange
            data_odom2.pose.orientation.z = 0.0            # unchange
            data_odom2.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom2)
            distance_from_origin = moved_distance+current_y_position
            final_x_position = data_odom2.pose.position.x
            final_z_position = data_odom2.pose.position.z

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(distance_from_origin)        # change
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange

            check = False

        elif choice == 3:
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The current x-axis position is : "+ str(current_position.x)
            print "The current y-axis position is : "+ str(current_position.y)
            print "The current z-axis position is : "+ str(current_position.z)
            print "The previous LeftEncoder_value is : "+ str(saved_left)
            print "The previous RightEncoder_value is : "+ str(saved_right)

            check = False

        elif choice == 4:
            data_odom4 = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The current x-axis position is : "+ str(current_x_position)
            print "The current y-axis position is : "+ str(current_y_position)
            print "The current z-axis position is : "+ str(current_z_position)
            print "The previous LeftEncoder_value is : "+ str(saved_left)
            print "The previous RightEncoder_value is : "+ str(saved_right)

            # calculate the distance
            left_wheel_diff = float(current_left-saved_left)
            right_wheel_diff = float(current_right-saved_right)
            left_wheel_moved = (left_wheel_diff/encoder_one_rotation)*wheel_length
            right_wheel_moved = (right_wheel_diff/encoder_one_rotation)*wheel_length
            moved_distance =  (right_wheel_moved+left_wheel_moved)/2.0
            print "The moved distance (cm) is : "+ str(moved_distance)

            # prepare the required information for set_model_state topic
            # Header
            data_odom4.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom4.pose.position.x = 0.0                  # should be unchange
            data_odom4.pose.position.y = -(moved_distance/100)     # change
            data_odom4.pose.position.z = 0.0               # unchange

            # orientation
            data_odom4.pose.orientation.x = 0.0               # unchange
            data_odom4.pose.orientation.y = 0.0               # unchange
            data_odom4.pose.orientation.z = 0.0            # unchange
            data_odom4.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom4)
            distance_from_origin = -(moved_distance+current_y_position)
            final_x_position = data_odom4.pose.position.x
            final_z_position = data_odom4.pose.position.z

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(distance_from_origin)        # change
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange

            check = False
        elif choice == 5:
            data_odom5 = ModelState()
            data_odom5.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom5.pose.position.x = 0.0                  # should be unchange
            data_odom5.pose.position.y = 0.0     # change
            data_odom5.pose.position.z = 0.0               # unchange

            # orientation
            data_odom5.pose.orientation.x = 0.0               # unchange
            data_odom5.pose.orientation.y = 0.0               # unchange
            data_odom5.pose.orientation.z = 0.0            # unchange
            data_odom5.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom5)
            final_x_position = data_odom5.pose.position.x
            final_y_position = data_odom5.pose.position.y
            final_z_position = data_odom5.pose.position.z

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # change
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "Success!"


        elif choice == 6:
            check = True

    while check == True:
        sys.exit()

