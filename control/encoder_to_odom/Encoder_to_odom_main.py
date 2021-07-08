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
    check = False
    rospy.init_node('odometry', anonymous=True)
    while check == False:
        saved_x_position = distance_from_x_origin
        saved_x_position_negative = distance_from_x_origin
        saved_y_position = distance_from_y_origin
        saved_y_position_negative = distance_from_y_origin
        rospy.Subscriber("/LeftEncoder_value", Int64, callbackL)    #subscribe the rostopic "LeftEncoder_value"
        rospy.Subscriber("/RightEncoder_value", Int64, callbackR)   #subscribe the rostopic "RightEncoder_value"
        rospy.Subscriber("/ground_truth/state", Odometry, fake_callback)   #subscribe the rostopic "RightEncoder_value"
        updateodom = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        saved_left = current_left
        saved_right = current_right
        print "Please choose your robot motion"
        print "Before control the car, please place the cursor to other window!"
        print "1. moving forward (press W two times immediately)"
        print "2. moving backward (press S two times immediately)"
        print "3. rotating to left 90 degree (press A three times immediately)"
        print "4. rotating to right 90 degree (press D three times immediately)"
        print "5. moving to left"
        print "6. moving to right"
        print "7. back to the origin"
        print "8. Exit"

        choice = int(raw_input("Enter a choice: "))
        
        if choice == 1:
            data_odom = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The previous x-axis position is : "+ str(current_x_position*100.0)
            print "The previous y-axis position is : "+ str(current_y_position*100.0)
            print "The previous z-axis position is : "+ str(current_z_position)
            print "The current z-axis orientation is : "+ str(final_z_position)
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
            data_odom.pose.position.x = 0.0                  # unchange
            data_odom.pose.position.y = (moved_distance+saved_y_position)/100.0     # change
            data_odom.pose.position.z = 0.0               # unchange

            # orientation
            data_odom.pose.orientation.x = 0.0               # unchange
            data_odom.pose.orientation.y = 0.0               # unchange
            data_odom.pose.orientation.z = 0.0            # unchange
            data_odom.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            updateodom.publish(data_odom)
            distance_from_y_origin = moved_distance+saved_y_position
            final_x_position = data_odom.pose.position.x
            final_z_position = data_odom.pose.position.z

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(distance_from_y_origin)        # change
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            check = False

        elif choice == 2:
            data_odom2 = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The previous x-axis position is : "+ str(current_x_position*100.0)
            print "The previous y-axis position is : "+ str(current_y_position*100.0)
            print "The previous z-axis position is : "+ str(current_z_position)
            print "The current z-axis orientation is : "+ str(current_z_orientation)
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
            moved_distance = (0.0-moved_distance)
            print "The moved distance (cm) is : "+ str(moved_distance)


            # prepare the required information for set_model_state topic
            # Header
            data_odom2.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom2.pose.position.x = 0.0                  # unchange
            data_odom2.pose.position.y = (moved_distance+saved_y_position_negative)/100.0     # change
            data_odom2.pose.position.z = 0.0               # unchange

            # orientation
            data_odom2.pose.orientation.x = 0.0               # unchange
            data_odom2.pose.orientation.y = 0.0               # unchange
            data_odom2.pose.orientation.z = 0.0            # unchange
            data_odom2.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            updateodom.publish(data_odom2)
            distance_from_y_origin = moved_distance+saved_y_position_negative
            final_x_position = data_odom2.pose.position.x
            final_z_position = data_odom2.pose.position.z

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(distance_from_y_origin)        # change
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange

            check = False

        elif choice == 3:
            data_odom3 = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The previous x-axis position is : "+ str(current_x_position*100.0)
            print "The previous y-axis position is : "+ str(distance_from_y_origin)
            print "The previous z-axis position is : "+ str(current_z_position)
            print "The previous x-axis orientation is : "+ str(current_x_orientation)
            print "The previous y-axis orientation is : "+ str(current_y_orientation)
            print "The previous z-axis orientation is : "+ str(current_z_orientation)
            print "The previous LeftEncoder_value is : "+ str(saved_left)
            print "The previous RightEncoder_value is : "+ str(saved_right)
            print "The saved x-axis distance is : "+ str(saved_x_position)
            print "The saved y-axis distance is : "+ str(saved_y_position)

            # calculate the distance
            left_wheel_diff = float(current_left-saved_left)
            right_wheel_diff = float(current_right-saved_right)
            left_wheel_moved = (left_wheel_diff/encoder_one_rotation)*wheel_length
            right_wheel_moved = (right_wheel_diff/encoder_one_rotation)*wheel_length
            moved_distance = (right_wheel_moved+left_wheel_moved)/2.0
            moved_angular_distance = moved_distance/(current_z_orientation)
            print "The moved distance (cm) is : "+ str(moved_distance)
            print "The rotated distance (cm) is : "+ str(moved_angular_distance)
            moved_radian = moved_angular_distance/143.345

            # limit the z-axis orientation not larger than 1
            if moved_angular_distance < 41.0:  # rotate to left < 90 degree
                moved_radian = init_orientation+moved_radian
            elif moved_angular_distance > 41.0 and moved_angular_distance < 61.5 :  # rotate to left 90-135 degree
                moved_radian = (1.0-(moved_radian-1.0))


            # prepare the required information for set_model_state topic
            # Header
            data_odom3.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom3.pose.position.x = current_x_position                  # change
            data_odom3.pose.position.y = current_y_position                # change
            data_odom3.pose.position.z = 0.0               # unchange

            # orientation
            data_odom3.pose.orientation.x = 0.0               # unchange
            data_odom3.pose.orientation.y = 0.0               # unchange
            data_odom3.pose.orientation.z = moved_radian            # change
            data_odom3.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            updateodom.publish(data_odom3)
            final_x_orientation = data_odom3.pose.orientation.x
            final_y_orientation = data_odom3.pose.orientation.y
            final_z_orientation = data_odom3.pose.orientation.z
            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(distance_from_y_origin)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(final_z_orientation)        # change

            check = False

        elif choice == 4:
            data_odom4 = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The previous x-axis position is : "+ str(current_x_position*100.0)
            print "The previous y-axis position is : "+ str(distance_from_y_origin)
            print "The previous z-axis position is : "+ str(current_z_position)
            print "The previous x-axis orientation is : "+ str(current_x_orientation)
            print "The previous y-axis orientation is : "+ str(current_y_orientation)
            print "The previous z-axis orientation is : "+ str(current_z_orientation)
            print "The previous LeftEncoder_value is : "+ str(saved_left)
            print "The previous RightEncoder_value is : "+ str(saved_right)
            print "The saved x-axis distance is : "+ str(saved_x_position)
            print "The saved y-axis distance is : "+ str(saved_y_position)

            # calculate the distance
            left_wheel_diff = float(current_left-saved_left)
            right_wheel_diff = float(current_right-saved_right)
            left_wheel_moved = (left_wheel_diff/encoder_one_rotation)*wheel_length
            right_wheel_moved = (right_wheel_diff/encoder_one_rotation)*wheel_length
            moved_distance = (right_wheel_moved+left_wheel_moved)/2.0
            moved_angular_distance = moved_distance/(current_z_orientation)
            print "The moved distance (cm) is : "+ str(moved_distance)
            print "The rotated distance (cm) is : "+ str(moved_angular_distance)
            moved_radian = moved_angular_distance/143.345

            # limit the z-axis orientation not larger than 1
            if moved_angular_distance < 41.0:  # rotate to left < 90 degree
                moved_radian = (0.0-(init_orientation+moved_radian))


            # prepare the required information for set_model_state topic
            # Header
            data_odom4.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom4.pose.position.x = current_x_position                  # change
            data_odom4.pose.position.y = current_y_position                # change
            data_odom4.pose.position.z = 0.0               # unchange

            # orientation
            data_odom4.pose.orientation.x = 0.0               # unchange
            data_odom4.pose.orientation.y = 0.0               # unchange
            data_odom4.pose.orientation.z = moved_radian            # change
            data_odom4.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            updateodom.publish(data_odom4)
            final_x_orientation = data_odom4.pose.orientation.x
            final_y_orientation = data_odom4.pose.orientation.y
            final_z_orientation = data_odom4.pose.orientation.z
            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(distance_from_y_origin)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(final_z_orientation)        # change
            check = False

        if choice == 5:
            data_odom5 = ModelState()
            print "The current LeftEncoder_value is : "+ str(current_left)
            print "The current RightEncoder_value is : "+ str(current_right)
            print "The previous x-axis position is : "+ str(current_x_position*100.0)
            print "The previous y-axis position is : "+ str(current_y_position*100.0)
            print "The previous z-axis position is : "+ str(current_z_position)
            print "The previous z-axis orientation is : "+ str(final_z_orientation)
            print "The previous LeftEncoder_value is : "+ str(saved_left)
            print "The previous RightEncoder_value is : "+ str(saved_right)
            print "The saved x-axis distance is : "+ str(saved_x_position_negative)
            print "The saved y-axis distance is : "+ str(saved_y_position_negative)

            # calculate the distance
            left_wheel_diff = float(current_left-saved_left)
            right_wheel_diff = float(current_right-saved_right)
            left_wheel_moved = (left_wheel_diff/encoder_one_rotation)*wheel_length
            right_wheel_moved = (right_wheel_diff/encoder_one_rotation)*wheel_length
            moved_distance =  (right_wheel_moved+left_wheel_moved)/2.0
            moved_distance = (0.0-moved_distance)
            print "The moved distance (cm) is : "+ str(moved_distance)


            # prepare the required information for set_model_state topic
            # Header
            data_odom5.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom5.pose.position.x = (moved_distance+saved_x_position_negative)/100.0                  # change
            data_odom5.pose.position.y = saved_y_position/100     # unchange
            data_odom5.pose.position.z = 0.0               # unchange

            # orientation
            data_odom5.pose.orientation.x = 0.0               # unchange
            data_odom5.pose.orientation.y = 0.0               # unchange
            data_odom5.pose.orientation.z = final_z_orientation          # change
            data_odom5.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            updateodom.publish(data_odom5)
            distance_from_x_origin = moved_distance+saved_x_position_negative
            final_y_position = data_odom5.pose.position.y
            final_z_position = data_odom5.pose.position.z

            print "The final x-axis position (cm) is : "+ str(distance_from_x_origin)        # change
            print "The final y-axis position (cm) is : "+ str(saved_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(final_z_orientation)        # unchange
            check = False

        if choice == 6:
            data_odom6 = ModelState()
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
            data_odom6.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom6.pose.position.x = (moved_distance+saved_x_position)/100.0                  # change
            data_odom6.pose.position.y = saved_y_position/100.0     # unchange
            data_odom6.pose.position.z = 0.0               # unchange

            # orientation
            data_odom6.pose.orientation.x = 0.0               # unchange
            data_odom6.pose.orientation.y = 0.0               # unchange
            data_odom6.pose.orientation.z = final_z_orientation            # change
            data_odom6.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            updateodom.publish(data_odom6)
            distance_from_x_origin = moved_distance+saved_x_position
            final_z_position = data_odom6.pose.position.z

            print "The final x-axis position (cm) is : "+ str(distance_from_x_origin)        # change
            print "The final y-axis position (cm) is : "+ str(saved_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(final_z_orientation)        # unchange
            check = False

        elif choice == 7:
            data_odom7 = ModelState()
            data_odom7.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom7.pose.position.x = 0.0                  # unchange
            data_odom7.pose.position.y = 0.0     # change
            data_odom7.pose.position.z = 0.0               # unchange

            # orientation
            data_odom7.pose.orientation.x = 0.0               # unchange
            data_odom7.pose.orientation.y = 0.0               # unchange
            data_odom7.pose.orientation.z = 0.0            # unchange
            data_odom7.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom7)
            final_x_position = data_odom7.pose.position.x
            final_y_position = data_odom7.pose.position.y
            final_z_position = data_odom7.pose.position.z
            final_x_orientation = data_odom7.pose.orientation.x
            final_y_orientation = data_odom7.pose.orientation.y
            final_z_orientation = data_odom7.pose.orientation.z
            final_w_orientation = data_odom7.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(final_z_orientation)        # unchange
            print "Success!"


        elif choice == 8:
            check = True

    while check == True:
        sys.exit()

