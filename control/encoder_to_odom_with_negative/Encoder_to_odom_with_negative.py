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
from std_msgs.msg import String, Duration

class OdomEncoder:
    def __init__(self):
        self.encoder_one_rotation = 1854.0
        self.wheel_length = 19.0       #15.7079632679 is come from my calculation but 16.0 is more suitable to simulation
        self.current_left = 0
        self.current_right = 0
        self.current_x_position = 0.0
        self.current_y_position = 0.0
        self.current_z_position = 0.0
        self.current_x_orientation = 0.0
        self.current_y_orientation = 0.0
        self.current_z_orientation = 0.0
        self.current_z_orientation = 0.0
        self.saved_x_position = 0.0
        self.saved_y_position = 0.0
        self.saved_z_position = 0.0
        self.final_x_position = 0.0
        self.final_y_position = 0.0
        self.final_z_position = 0.0
        self.distance_from_x_origin = 0.0
        self.distance_from_y_origin = 0.0
        self.updateodom = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.saved_left = 0.0
        self.saved_right = 0.0
        self.moved_radian = 0.0
        self.abs_moved_radian = 0.0
        self.saved_moved_radian = 0.0
        self.moved_distance = 0.0
        self.compensated_parameter = 0.23
        self.left_encoder_sub = rospy.Subscriber("/LeftEncoder_value", Int64,self.callbackL)    #subscribe the rostopic "LeftEncoder_value"
        self.right_encoder_sub = rospy.Subscriber("/RightEncoder_value", Int64, self.callbackR)   #subscribe the rostopic "RightEncoder_value"
        self.ground_truth_sub = rospy.Subscriber("/ground_truth/state", Odometry, self.fake_callback)   #subscribe the rostopic "/ground_truth/state"

    def callbackL(self, data):
        self.current_left = int(data.data)                             #print the received message from rostopic

    def callbackR(self, data):
        self.current_right = int(data.data)                              #print the received message from rostopic

    def fake_callback(self, data):
        self.current_x_position = float(data.pose.pose.position.x)
        self.current_y_position = float(data.pose.pose.position.y)
        self.current_z_position = float(data.pose.pose.position.z)
        self.current_x_orientation = float(data.pose.pose.orientation.x)
        self.current_y_orientation = float(data.pose.pose.orientation.y)
        self.current_z_orientation = float(data.pose.pose.orientation.z)
        self.current_w_orientation = float(data.pose.pose.orientation.z)

    def update(self):
        rospy.init_node('encoder_to_odom', anonymous=True)
        self.saved_left = float(self.current_left)
        self.saved_right = float(self.current_right)
        self.saved_x_position = float(self.distance_from_x_origin)
        self.saved_y_position = float(self.distance_from_y_origin)

    def pose(self):
        #print the message 
        print "The current LeftEncoder_value is : "+ str(self.current_left)
        print "The current RightEncoder_value is : "+ str(self.current_right)
        print "The previous x-axis position is : "+ str(self.current_x_position*100.0)
        print "The previous y-axis position is : "+ str(self.current_y_position*100.0)
        print "The previous z-axis position is : "+ str(self.current_z_position)
        print "The current z-axis orientation is : "+ str(self.current_z_orientation)
        print "The previous LeftEncoder_value is : "+ str(self.saved_left)
        print "The previous RightEncoder_value is : "+ str(self.saved_right)
        print "The saved x-axis distance is : "+ str(self.saved_x_position)
        print "The saved y-axis distance is : "+ str(self.saved_y_position)

    def calculation(self):
        # calculate the distance and orientation

        # forward mode
        if (self.current_left > self.saved_left) & (self.current_right > self.saved_right):
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            print "The moved distance (forward) (cm) is : "+ str(self.moved_distance)

        # backward mode
        elif (self.saved_left > self.current_left) & (self.saved_right > self.current_right):
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            print "The moved distance (backward) (cm) is : "+ str(self.moved_distance)

        # turn left
        elif (self.saved_left > self.current_left) & (self.current_right > self.saved_right):
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            self.moved_angular_distance = abs(self.moved_distance*self.moved_distance)
            print "The moved distance (cm) is : "+ str(abs(self.moved_distance))
            print "The rotated distance (turn left) (cm) is : "+ str(abs(self.moved_angular_distance))
            self.moved_radian = abs(self.moved_angular_distance)
            self.abs_moved_radian = abs(self.moved_radian)

        # turn right
        elif (self.current_left > self.saved_left) & (self.saved_right > self.current_right):
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            self.moved_angular_distance = abs(self.moved_distance*self.moved_distance)
            print "The moved distance (cm) is : "+ str(abs(self.moved_distance))
            print "The rotated distance (turn right) (cm) is : "+ str(abs(self.moved_angular_distance))
            self.moved_radian = abs(self.moved_angular_distance)
            self.abs_moved_radian = abs(self.moved_radian)

        # stop
        elif (self.saved_left == self.current_left) & (self.current_right == self.saved_right):
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            self.moved_angular_distance = self.moved_distance/(self.current_z_orientation)
            print "The moved distance (cm) is : "+ str(self.moved_distance)
            print "Stopped !"

    def publish(self):
        #self.updateodom.publish()
        self.data_odom = ModelState()
        self.data_odom.model_name = 'robot'
        #data_odom.header.frame_id = "world"
        # from geometry_msgs.msg.TransformStamped()
        #data_frame.child_frame_id = "base_footprint"

        # forward mode
        if (self.current_left > self.saved_left) & (self.current_right > self.saved_right):

            if (self.current_z_orientation < 0.005) or (self.current_z_orientation > 0.001):
                # pose
                self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

                self.data_odom.pose.position.x = self.current_x_position       # unchange
                self.data_odom.pose.position.y = self.distance_from_y_origin/100.0     # change
                self.data_odom.pose.position.z = 0.05               # unchange

                # orientation
                self.data_odom.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom.pose.orientation.z = abs(self.current_z_orientation)            # unchange
                self.data_odom.pose.orientation.w = 1.0            # unchange

                # update the setmodelstate 
                self.updateodom.publish(self.data_odom)
                self.final_x_position = self.current_x_position
                self.final_z_position = self.current_z_position

                print "The final x-axis position (forward) (cm) is : "+ str(self.final_x_position)        # unchange
                print "The final y-axis position (forward) (cm) is : "+ str(self.distance_from_y_origin)        # change
                print "The final z-axis position (forward) (cm) is : "+ str(self.final_z_position)        # unchange

            # move to left
            if self.current_z_orientation > 0.25:
                self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)

                self.data_odom.pose.position.x = (-self.distance_from_x_origin)/100.0       # change
                self.data_odom.pose.position.y = self.current_y_position     # unchange
                self.data_odom.pose.position.z = 0.05               # unchange

                # orientation
                self.data_odom.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom.pose.orientation.z = abs(self.current_z_orientation)+self.compensated_parameter            # unchange
                self.data_odom.pose.orientation.w = 1.0            # unchange

                # update the setmodelstate 
                self.updateodom.publish(self.data_odom)
                self.final_y_position = self.current_y_position
                self.final_z_position = self.current_z_position

                print "The final x-axis position (left) (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (left) (cm) is : "+ str(self.final_y_position)        # unchange
                print "The final z-axis position (left) (cm) is : "+ str(self.final_z_position)        # unchange

            # move to right
            if self.current_z_orientation < -0.25:
                self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)

                self.data_odom.pose.position.x = self.distance_from_x_origin/100.0       # change
                self.data_odom.pose.position.y = self.current_y_position     # unchange
                self.data_odom.pose.position.z = 0.05               # unchange

                # orientation
                self.data_odom.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom.pose.orientation.z = self.current_z_orientation-self.compensated_parameter            # unchange
                self.data_odom.pose.orientation.w = 1.0            # unchange

                # update the setmodelstate 
                self.updateodom.publish(self.data_odom)
                self.final_y_position = self.current_y_position
                self.final_z_position = self.current_z_position

                print "The final x-axis position (right) (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (right) (cm) is : "+ str(self.final_y_position)        # unchange
                print "The final z-axis position (right) (cm) is : "+ str(self.final_z_position)        # unchange

        # backward mode
        elif (self.saved_left > self.current_left) & (self.saved_right > self.current_right):

            # pose
            self.distance_from_y_origin = self.saved_y_position+self.moved_distance

            self.data_odom.pose.position.x = self.current_x_position                  # unchange
            self.data_odom.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom.pose.position.z = 0.05               # unchange

            # orientation
            self.data_odom.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom.pose.orientation.z = abs(self.current_z_orientation)            # unchange
            self.data_odom.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom)
            self.final_x_position = self.current_x_position
            self.final_z_position = self.current_z_position

            print "The final x-axis position (backward) (cm) is : "+ str(self.final_x_position)        # unchange
            print "The final y-axis position (backward) (cm) is : "+ str(self.distance_from_y_origin)        # change
            print "The final z-axis position (backward) (cm) is : "+ str(self.final_z_position)        # unchange

        # turn left
        elif (self.saved_left > self.current_left) & (self.current_right > self.saved_right):

            self.moved_radian = abs(self.saved_moved_radian)+abs(self.abs_moved_radian*17)
            self.saved_moved_radian = self.moved_radian

            # pose
            self.data_odom.pose.position.x = self.current_x_position                  # change
            self.data_odom.pose.position.y = self.current_y_position                # change
            self.data_odom.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom.pose.orientation.x = 0.0               # unchange
            self.data_odom.pose.orientation.y = 0.0               # unchange
            self.data_odom.pose.orientation.z = self.moved_radian            # change
            self.data_odom.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom)
            self.final_x_orientation = self.data_odom.pose.orientation.x
            self.final_y_orientation = self.data_odom.pose.orientation.y
            self.final_z_orientation = self.data_odom.pose.orientation.z
            print "The final x-axis position (cm) is : "+ str(self.final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # unchange
            print "The final z-axis position (cm) is : "+ str(self.final_z_position)        # unchange
            print "The final x-axis orientation (anticlockwise) is : "+ str(self.final_x_orientation)        # unchange
            print "The final y-axis orientation (anticlockwise) is : "+ str(self.final_y_orientation)        # unchange
            print "The final z-axis orientation (anticlockwise) is : "+ str(self.final_z_orientation)        # change


        # turn right
        elif (self.current_left > self.saved_left) & (self.saved_right > self.current_right):
            self.moved_radian = abs(self.saved_moved_radian)+abs(self.abs_moved_radian*15)
            self.moved_radian = -self.moved_radian
            self.saved_moved_radian = self.moved_radian

            print (self.moved_radian)
            # pose
            self.data_odom.pose.position.x = self.current_x_position                  # change
            self.data_odom.pose.position.y = self.current_y_position                # change
            self.data_odom.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom.pose.orientation.x = 0.0               # unchange
            self.data_odom.pose.orientation.y = 0.0               # unchange
            self.data_odom.pose.orientation.z = self.moved_radian            # change
            self.data_odom.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom)
            self.final_x_orientation = self.data_odom.pose.orientation.x
            self.final_y_orientation = self.data_odom.pose.orientation.y
            self.final_z_orientation = self.data_odom.pose.orientation.z
            print "The final x-axis position (cm) is : "+ str(self.final_x_position)        # unchange
            print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # unchange
            print "The final z-axis position (cm) is : "+ str(self.final_z_position)        # unchange
            print "The final x-axis orientation (clockwise) is : "+ str(self.final_x_orientation)        # unchange
            print "The final y-axis orientation (clockwise) is : "+ str(self.final_y_orientation)        # unchange
            print "The final z-axis orientation (clockwise) is : "+ str(self.final_z_orientation)        # change


if __name__ == "__main__":
        try:
            odom = OdomEncoder()
            while not rospy.is_shutdown():
                odom.update()
                rospy.sleep(rospy.Duration(0.1))
                odom.pose()
                odom.calculation()
                odom.publish()
        except rospy.ROSInterruptException:
            pass