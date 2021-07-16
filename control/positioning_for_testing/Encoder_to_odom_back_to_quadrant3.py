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
        print "1. Back to the origin facing forward"
        print "2. Back to the origin facing backward"
        print "3. Back to the origin facing to left"
        print "4. Back to the origin facing to right"
        print "5. Back to the origin facing quadrant 1"
        print "6. Back to the origin facing quadrant 2"
        print "7. Back to the origin facing quadrant 3"
        print "8. Back to the origin facing quadrant 4"
        print "9. Exit"

        choice = int(raw_input("Enter a choice: "))
        if choice == 1:
            data_odom = ModelState()
            data_odom.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom.pose.position.x = -5.0                  # unchange
            data_odom.pose.position.y = -5.0     # change
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

        if choice == 2:
            data_odom2 = ModelState()
            data_odom2.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom2.pose.position.x = -5.0                  # unchange
            data_odom2.pose.position.y = -5.0     # change
            data_odom2.pose.position.z = 0.0               # unchange

            # orientation
            data_odom2.pose.orientation.x = 0.0               # unchange
            data_odom2.pose.orientation.y = 0.0               # unchange
            data_odom2.pose.orientation.z = -10000000000000000000000000.0            # unchange
            data_odom2.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom2)
            final_x_position = data_odom2.pose.position.x
            final_y_position = data_odom2.pose.position.y
            final_z_position = data_odom2.pose.position.z
            final_x_orientation = data_odom2.pose.orientation.x
            final_y_orientation = data_odom2.pose.orientation.y
            final_z_orientation = data_odom2.pose.orientation.z
            final_w_orientation = data_odom2.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(-1.0)        # unchange
            print "Success!"
            check = False

        if choice == 3:
            data_odom3 = ModelState()
            data_odom3.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom3.pose.position.x = -5.0                  # unchange
            data_odom3.pose.position.y = -5.0     # change
            data_odom3.pose.position.z = 0.0               # unchange

            # orientation
            data_odom3.pose.orientation.x = 0.0               # unchange
            data_odom3.pose.orientation.y = 0.0               # unchange
            data_odom3.pose.orientation.z = 1.0            # unchange
            data_odom3.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom3)
            final_x_position = data_odom3.pose.position.x
            final_y_position = data_odom3.pose.position.y
            final_z_position = data_odom3.pose.position.z
            final_x_orientation = data_odom3.pose.orientation.x
            final_y_orientation = data_odom3.pose.orientation.y
            final_z_orientation = data_odom3.pose.orientation.z
            final_w_orientation = data_odom3.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(0.707)        # unchange
            print "Success!"
            check = False

        if choice == 4:
            data_odom4 = ModelState()
            data_odom4.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom4.pose.position.x = -5.0                  # unchange
            data_odom4.pose.position.y = -5.0     # change
            data_odom4.pose.position.z = 0.0               # unchange

            # orientation
            data_odom4.pose.orientation.x = 0.0               # unchange
            data_odom4.pose.orientation.y = 0.0               # unchange
            data_odom4.pose.orientation.z = -1.0            # unchange
            data_odom4.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom4)
            final_x_position = data_odom4.pose.position.x
            final_y_position = data_odom4.pose.position.y
            final_z_position = data_odom4.pose.position.z
            final_x_orientation = data_odom4.pose.orientation.x
            final_y_orientation = data_odom4.pose.orientation.y
            final_z_orientation = data_odom4.pose.orientation.z
            final_w_orientation = data_odom4.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(-0.707)        # unchange
            print "Success!"
            check = False

        if choice == 5:
            data_odom5 = ModelState()
            data_odom5.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom5.pose.position.x = -5.0                  # unchange
            data_odom5.pose.position.y = -5.0     # change
            data_odom5.pose.position.z = 0.0               # unchange

            # orientation
            data_odom5.pose.orientation.x = 0.0               # unchange
            data_odom5.pose.orientation.y = 0.0               # unchange
            data_odom5.pose.orientation.z = -0.41215            # unchange
            data_odom5.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom5)
            final_x_position = data_odom5.pose.position.x
            final_y_position = data_odom5.pose.position.y
            final_z_position = data_odom5.pose.position.z
            final_x_orientation = data_odom5.pose.orientation.x
            final_y_orientation = data_odom5.pose.orientation.y
            final_z_orientation = data_odom5.pose.orientation.z
            final_w_orientation = data_odom5.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(0.38227934405)        # unchange
            print "Success!"
            check = False

        if choice == 6:
            data_odom6 = ModelState()
            data_odom6.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom6.pose.position.x = -5.0                  # unchange
            data_odom6.pose.position.y = -5.0     # change
            data_odom6.pose.position.z = 0.0               # unchange

            # orientation
            data_odom6.pose.orientation.x = 0.0               # unchange
            data_odom6.pose.orientation.y = 0.0               # unchange
            data_odom6.pose.orientation.z = 0.41485            # unchange
            data_odom6.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom6)
            final_x_position = data_odom6.pose.position.x
            final_y_position = data_odom6.pose.position.y
            final_z_position = data_odom6.pose.position.z
            final_x_orientation = data_odom6.pose.orientation.x
            final_y_orientation = data_odom6.pose.orientation.y
            final_z_orientation = data_odom6.pose.orientation.z
            final_w_orientation = data_odom6.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(-0.38227934405)        # unchange
            print "Success!"
            check = False

        if choice == 7:
            data_odom7 = ModelState()
            data_odom7.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom7.pose.position.x = -5.0                  # unchange
            data_odom7.pose.position.y = -5.0     # change
            data_odom7.pose.position.z = 0.0               # unchange

            # orientation
            data_odom7.pose.orientation.x = 0.0               # unchange
            data_odom7.pose.orientation.y = 0.0               # unchange
            data_odom7.pose.orientation.z = 2.405            # unchange
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
            print "The final z-axis orientation is : "+ str(0.923346114714)        # unchange
            print "Success!"
            check = False

        if choice == 8:
            data_odom8 = ModelState()
            data_odom8.model_name = 'robot'
            #data_odom.header.frame_id = "world"
            # from geometry_msgs.msg.TransformStamped()
            #data_frame.child_frame_id = "base_footprint"

            # pose
            data_odom8.pose.position.x = -5.0                  # unchange
            data_odom8.pose.position.y = -5.0     # change
            data_odom8.pose.position.z = 0.0               # unchange

            # orientation
            data_odom8.pose.orientation.x = 0.0               # unchange
            data_odom8.pose.orientation.y = 0.0               # unchange
            data_odom8.pose.orientation.z = -2.407            # unchange
            data_odom8.pose.orientation.w = 1.0            # unchange

            # update the groundtruth 
            updateodom.publish(data_odom8)
            final_x_position = data_odom8.pose.position.x
            final_y_position = data_odom8.pose.position.y
            final_z_position = data_odom8.pose.position.z
            final_x_orientation = data_odom8.pose.orientation.x
            final_y_orientation = data_odom8.pose.orientation.y
            final_z_orientation = data_odom8.pose.orientation.z
            final_w_orientation = data_odom8.pose.orientation.w

            print "The final x-axis position (cm) is : "+ str(final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(final_y_position)        # unchange
            print "The final z-axis position (cm) is : "+ str(final_z_position)        # unchange
            print "The final x-axis orientation is : "+ str(final_x_orientation)        # unchange
            print "The final y-axis orientation is : "+ str(final_y_orientation)        # unchange
            print "The final z-axis orientation is : "+ str(-0.923346114714)        # unchange
            print "Success!"
            check = False

        elif choice == 9:
            check = True
    while check == True:
        sys.exit()