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
from std_msgs.msg import String, Duration, Header


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
        self.boundary_parameter = 0.1
        self.negative_boundary_parameter = -0.1
        self.leave_origin = False
        self.quadrant = 0
        self.positive_x_origin = False
        self.positive_y_origin = False
        self.negative_x_origin = False
        self.negative_y_origin = False
        self.positive_quadrant1_origin = False
        self.negative_quadrant1_origin = False
        self.positive_quadrant2_origin = False
        self.negative_quadrant2_origin = False
        self.positive_quadrant3_origin = False
        self.negative_quadrant3_origin = False
        self.positive_quadrant4_origin = False
        self.negative_quadrant4_origin = False
        self.turn_left = False
        self.turn_right = False
        self.move_forward = False
        self.move_backward = False
        self.over_half = 1
        self.leave_origin_orientation_z = 0.0
        self.move_crossline = False
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
        if (self.current_z_orientation < 0.06)&(self.current_z_orientation > -0.06):
            self.over_half = 1
    def pose(self):
        #print the message 
        print "The current LeftEncoder_value is : "+ str(self.current_left)
        print "The current RightEncoder_value is : "+ str(self.current_right)
        print "The previous x-axis position is : "+ str(self.current_x_position*100.0)
        print "The previous y-axis position is : "+ str(self.current_y_position*100.0)
        print "The current z-axis orientation is : "+ str(self.current_z_orientation)
        print "The previous LeftEncoder_value is : "+ str(self.saved_left)
        print "The previous RightEncoder_value is : "+ str(self.saved_right)
        print "The saved x-axis distance is : "+ str(self.saved_x_position)
        print "The saved y-axis distance is : "+ str(self.saved_y_position)

    def calculation(self):
        # calculate the distance and orientation

        # forward mode
        if (self.current_left > self.saved_left) & (self.current_right > self.saved_right):
            self.turn_right = False
            self.turn_left = False
            self.move_forward = True
            self.move_backward = False
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            print "The moved distance (forward) (cm) is : "+ str(self.moved_distance)
            print ""

        # backward mode
        elif (self.saved_left > self.current_left) & (self.saved_right > self.current_right):
            self.turn_right = False
            self.turn_left = False
            self.move_forward = False
            self.move_backward = True
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            print "The moved distance (backward) (cm) is : "+ str(self.moved_distance)
            print ""

        # turn left
        elif (self.saved_left > self.current_left) & (self.current_right > self.saved_right):
            self.turn_right = False
            self.turn_left = True
            self.move_forward = False
            self.move_backward = False
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0

            print "The moved distance (cm) is : "+ str(abs(self.moved_distance))
            print "The rotated distance (turn left) (cm) is : "+ str(abs(self.moved_angular_distance))
            self.moved_radian = abs(self.moved_angular_distance)
            self.abs_moved_radian = abs(self.moved_radian)
            print ""

            # turn left parameter tuning when facing forward (from 180 degree to 360 degree)
            if (self.over_half == 0)&(self.current_z_orientation < 0.06)&(self.current_z_orientation > -0.707):
                self.moved_distance = self.moved_distance*5.4                                                   # previous 5.4
            elif (self.over_half == 0)&(self.current_z_orientation < -0.707)&(self.current_z_orientation > -0.757):
                self.moved_distance = self.moved_distance*6.413                                             # previous 6.413
            elif (self.over_half == 0)&(self.current_z_orientation < -0.757)&(self.current_z_orientation > -0.807):
                self.moved_distance = self.moved_distance*7.413                                             # previous 7.413
            elif (self.over_half == 0)&(self.current_z_orientation < -0.807)&(self.current_z_orientation > -0.857):
                self.moved_distance = self.moved_distance*7.413                                             # previous 7.413
            elif (self.over_half == 0)&(self.current_z_orientation < -0.857)&(self.current_z_orientation > -0.907):
                self.moved_distance = self.moved_distance*8.413                                             # previous 8.413
            elif (self.over_half == 0)&(self.current_z_orientation < -0.907)&(self.current_z_orientation > -0.927):
                self.moved_distance = self.moved_distance*10.413                                             # previous 10.413
            elif (self.over_half == 0)&(self.current_z_orientation < -0.927)&(self.current_z_orientation > -0.957):
                self.moved_distance = self.moved_distance*11.413                                             # previous 11.413
            elif (self.over_half == 0)&(self.current_z_orientation < -0.957)&(self.current_z_orientation > -0.99):
                self.moved_distance = self.moved_distance*13.5                                             # previous 13.5
            elif (self.over_half == 0)&(self.current_z_orientation < -0.99)&(self.current_z_orientation > -0.999):
                self.moved_distance = self.moved_distance*13.5                                              # previous 13.5
            elif (self.over_half == 0)&(self.current_z_orientation < -0.999)&(self.current_z_orientation > -0.999999999):
                self.moved_distance = self.moved_distance*85.0                            # previous 85.0

            # turn left parameter tuning facing forward (from 0 degree to 180 degree)
            if (self.over_half == 1)&(self.current_z_orientation > 0.06)&(self.current_z_orientation < 0.707):
                self.moved_distance = self.moved_distance*7.4                                             # previous 5.4
            elif (self.over_half == 1)&(self.current_z_orientation > 0.707)&(self.current_z_orientation < 0.757):
                self.moved_distance = self.moved_distance*6.413                                             # previous 6.413
            elif (self.over_half == 1)&(self.current_z_orientation > 0.757)&(self.current_z_orientation < 0.807):
                self.moved_distance = self.moved_distance*6.413                                             # previous 6.413
            elif (self.over_half == 1)&(self.current_z_orientation > 0.807)&(self.current_z_orientation < 0.857):
                self.moved_distance = self.moved_distance*6.413                                             # previous 6.413
            elif (self.over_half == 1)&(self.current_z_orientation > 0.857)&(self.current_z_orientation < 0.907):
                self.moved_distance = self.moved_distance*7.413                                             # previous 7.413
            elif (self.over_half == 1)&(self.current_z_orientation > 0.907)&(self.current_z_orientation < 0.927):
                self.moved_distance = self.moved_distance*10.413                                             # previous 10.413
            elif (self.over_half == 1)&(self.current_z_orientation > 0.927)&(self.current_z_orientation < 0.957):
                self.moved_distance = self.moved_distance*11.413                                             # previous 11.413
            elif (self.over_half == 1)&(self.current_z_orientation > 0.957)&(self.current_z_orientation < 0.99):
                self.moved_distance = self.moved_distance*13.5                                             # previous 13.5

            # turn left parameter tuning facing backward (from 0 degree to 180 degree)
            self.moved_angular_distance = abs(self.moved_distance*self.moved_distance)
            print "The moved distance (cm) is : "+ str(abs(self.moved_distance))
            print "The rotated distance (turn left) (cm) is : "+ str(abs(self.moved_angular_distance))
            self.moved_radian = abs(self.moved_angular_distance)
            self.abs_moved_radian = abs(self.moved_radian)
            print ""
            # turn left parameter tuning facing forward
            if (self.over_half == 1)&(self.current_z_orientation > 0.999)&(self.current_z_orientation < 0.999999999):
                    self.moved_radian = -100.0                            # previous 1000000000000000000000
                    self.saved_moved_radian = -100.0
                    self.data_odom_right = ModelState()
                    self.data_odom_right.model_name = 'robot'
                    self.data_odom_right.pose.orientation.z = 100.0
                    self.updateodom.publish(self.data_odom_right)
                    self.over_half = 0
            # turn left parameter tuning facing forward
            elif (self.over_half == 1)&(self.current_z_orientation >0.99)&(self.current_z_orientation < 0.999):
                    self.moved_radian = -100.0                            # previous 1000000000000000000000
                    self.saved_moved_radian = -100.0
                    self.data_odom_right = ModelState()
                    self.data_odom_right.model_name = 'robot'
                    self.data_odom_right.pose.orientation.z = 100.0
                    self.updateodom.publish(self.data_odom_right)
                    self.over_half = 0

        # turn right
        elif (self.current_left > self.saved_left) & (self.saved_right > self.current_right):
            self.turn_right = True
            self.turn_left = False
            self.move_forward = False
            self.move_backward = False
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0

            # turn right parameter tuning when facing forward (from 0 degree to 180 degree)
            if (self.over_half == 1)&(self.current_z_orientation < 0.06)&(self.current_z_orientation > -0.707):
                    self.moved_distance = self.moved_distance*2.4                                                   # previous 3.4
            elif (self.over_half == 1)&(self.current_z_orientation < -0.707)&(self.current_z_orientation > -0.757):
                    self.moved_distance = self.moved_distance*3.413                                             # previous 3.413
            elif (self.over_half == 1)&(self.current_z_orientation < -0.757)&(self.current_z_orientation > -0.807):
                    self.moved_distance = self.moved_distance*4.413                                             # previous 4.413
            elif (self.over_half == 1)&(self.current_z_orientation < -0.807)&(self.current_z_orientation > -0.857):
                    self.moved_distance = self.moved_distance*4.413                                             # previous 4.413
            elif (self.over_half == 1)&(self.current_z_orientation < -0.857)&(self.current_z_orientation > -0.907):
                    self.moved_distance = self.moved_distance*5.413                                             # previous 5.413
            elif (self.over_half == 1)&(self.current_z_orientation < -0.907)&(self.current_z_orientation > -0.927):
                    self.moved_distance = self.moved_distance*8.413                                             # previous 8.413
            elif (self.over_half == 1)&(self.current_z_orientation < -0.927)&(self.current_z_orientation > -0.957):
                    self.moved_distance = self.moved_distance*9.413                                             # previous 9.413
            elif (self.over_half == 1)&(self.current_z_orientation < -0.957)&(self.current_z_orientation > -0.99):
                    self.moved_distance = self.moved_distance*11.5                                             # previous 11.5

            # turn right parameter tuning when facing forward (from 180 degree to 360 degree)
            if (self.over_half == 0)&(self.current_z_orientation > 0.06)&(self.current_z_orientation < 0.707):
                    self.moved_distance = self.moved_distance*3.4                                             # previous 3.4
            elif (self.over_half == 0)&(self.current_z_orientation > 0.707)&(self.current_z_orientation < 0.757):
                    self.moved_distance = self.moved_distance*3.413                                             # previous 3.413
            elif (self.over_half == 0)&(self.current_z_orientation > 0.757)&(self.current_z_orientation < 0.807):
                    self.moved_distance = self.moved_distance*4.413                                             # previous 4.413
            elif (self.over_half == 0)&(self.current_z_orientation > 0.807)&(self.current_z_orientation < 0.857):
                    self.moved_distance = self.moved_distance*4.413                                             # previous 4.413
            elif (self.over_half == 0)&(self.current_z_orientation > 0.857)&(self.current_z_orientation < 0.907):
                    self.moved_distance = self.moved_distance*5.413                                             # previous 5.413
            elif (self.over_half == 0)&(self.current_z_orientation > 0.907)&(self.current_z_orientation < 0.927):
                    self.moved_distance = self.moved_distance*8.413                                             # previous 8.413
            elif (self.over_half == 0)&(self.current_z_orientation > 0.927)&(self.current_z_orientation < 0.957):
                    self.moved_distance = self.moved_distance*9.413                                             # previous 9.413
            elif (self.over_half == 0)&(self.current_z_orientation > 0.957)&(self.current_z_orientation < 0.99):
                    self.moved_distance = self.moved_distance*11.5                                             # previous 11.5
            elif (self.over_half == 0)&(self.current_z_orientation > 0.99)&(self.current_z_orientation < 0.999):
                    self.moved_distance = self.moved_distance*11.5                                              # previous 1000
            elif (self.over_half == 0)&(self.current_z_orientation > 0.999)&(self.current_z_orientation < 0.999999999):
                    self.moved_distance = self.moved_distance*55.0                            # previous 1000000000000000000000

            self.moved_angular_distance = abs(self.moved_distance*self.moved_distance)
            print "The moved distance (cm) is : "+ str(abs(self.moved_distance))
            print "The rotated distance (turn right) (cm) is : "+ str(abs(self.moved_angular_distance))
            self.moved_radian = abs(self.moved_angular_distance)
            self.abs_moved_radian = abs(self.moved_radian)
            print ""

            # turn right parameter tuning when facing forward 
            if (self.over_half == 1)&(self.current_z_orientation < -0.999)&(self.current_z_orientation > -0.999999999):
                    self.moved_radian = 100.0                            # previous 1000000000000000000000
                    self.saved_moved_radian = 100.0
                    self.data_odom_right = ModelState()
                    self.data_odom_right.model_name = 'robot'
                    self.data_odom_right.pose.orientation.z = 100.0
                    self.updateodom.publish(self.data_odom_right)
                    self.over_half = 0
            # turn right parameter tuning when facing forward 
            elif (self.over_half == 1)&(self.current_z_orientation < -0.99)&(self.current_z_orientation > -0.999):
                    self.moved_radian = 100.0                            # previous 1000000000000000000000
                    self.saved_moved_radian = 100.0
                    self.data_odom_right = ModelState()
                    self.data_odom_right.model_name = 'robot'
                    self.data_odom_right.pose.orientation.z = 100.0
                    self.updateodom.publish(self.data_odom_right)
                    self.over_half = 0

        # stop
        elif (self.saved_left == self.current_left) & (self.current_right == self.saved_right):
            self.turn_right = False
            self.turn_left = False
            self.move_forward = False
            self.move_backward = False
            self.left_wheel_diff = float(self.current_left-self.saved_left)
            self.right_wheel_diff = float(self.current_right-self.saved_right)
            self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
            self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
            self.moved_angular_distance = self.moved_distance/(self.current_z_orientation)
            print "The moved distance (cm) is : "+ str(self.moved_distance)
            print "Stopped !"
            print ""

    def define_quadrant(self):
        # located in quadrant 1 (x>0.03)&(y>0.03) which means the quadrant define box size is 0.0125x0.0125
        if (self.current_x_position > (self.boundary_parameter-0.0875)) & (self.current_y_position > (self.boundary_parameter-0.0875)):
            self.quadrant = 1
            self.positive_quadrant1_origin = True
            self.negative_quadrant1_origin = False
            self.positive_quadrant2_origin = False
            self.negative_quadrant2_origin = False
            self.positive_quadrant3_origin = False
            self.negative_quadrant3_origin = True
            self.positive_quadrant4_origin = False
            self.negative_quadrant4_origin = False
            if ((self.current_z_orientation < -0.01) & (self.current_z_orientation > -0.697))or((self.current_z_orientation > 0.01) & (self.current_z_orientation < 0.697))or((self.current_z_orientation > 0.72) & (self.current_z_orientation < 0.99))or((self.current_z_orientation < -0.72) & (self.current_z_orientation > -0.99)):
                self.move_crossline = True
            elif ((self.current_z_orientation < 0.01) & (self.current_z_orientation > -0.01))or((self.current_z_orientation < -0.99) & (self.current_z_orientation > -1.0))or((self.current_z_orientation < -0.697) & (self.current_z_orientation > -0.72))or((self.current_z_orientation > 0.697) & (self.current_z_orientation < 0.72)):
                self.move_crossline = False
        # located in quadrant 2
        if (self.current_x_position < self.negative_boundary_parameter+0.0875) & (self.current_y_position > self.boundary_parameter-0.0875):
            self.quadrant = 2
            self.positive_quadrant1_origin = False
            self.negative_quadrant1_origin = False
            self.positive_quadrant2_origin = True
            self.negative_quadrant2_origin = False
            self.positive_quadrant3_origin = False
            self.negative_quadrant3_origin = False
            self.positive_quadrant4_origin = False
            self.negative_quadrant4_origin = True
            if ((self.current_z_orientation < -0.01) & (self.current_z_orientation > -0.697))or((self.current_z_orientation > 0.01) & (self.current_z_orientation < 0.697))or((self.current_z_orientation > 0.72) & (self.current_z_orientation < 0.99))or((self.current_z_orientation < -0.72) & (self.current_z_orientation > -0.99)):
                self.move_crossline = True
            elif ((self.current_z_orientation < 0.01) & (self.current_z_orientation > -0.01))or((self.current_z_orientation < -0.99) & (self.current_z_orientation > -1.0))or((self.current_z_orientation < -0.697) & (self.current_z_orientation > -0.72))or((self.current_z_orientation > 0.697) & (self.current_z_orientation < 0.72)):
                self.move_crossline = False
        # located in quadrant 3
        if (self.current_x_position < self.negative_boundary_parameter+0.0875) & (self.current_y_position < self.negative_boundary_parameter+0.0875):
            self.quadrant = 3
            self.positive_quadrant1_origin = False
            self.negative_quadrant1_origin = True
            self.positive_quadrant2_origin = False
            self.negative_quadrant2_origin = False
            self.positive_quadrant3_origin = True
            self.negative_quadrant3_origin = False
            self.positive_quadrant4_origin = False
            self.negative_quadrant4_origin = False
            if ((self.current_z_orientation < -0.01) & (self.current_z_orientation > -0.697))or((self.current_z_orientation > 0.01) & (self.current_z_orientation < 0.697))or((self.current_z_orientation > 0.72) & (self.current_z_orientation < 0.99))or((self.current_z_orientation < -0.72) & (self.current_z_orientation > -0.99)):
                self.move_crossline = True
            elif ((self.current_z_orientation < 0.01) & (self.current_z_orientation > -0.01))or((self.current_z_orientation < -0.99) & (self.current_z_orientation > -1.0))or((self.current_z_orientation < -0.697) & (self.current_z_orientation > -0.72))or((self.current_z_orientation > 0.697) & (self.current_z_orientation < 0.72)):
                self.move_crossline = False
        # located in quadrant 4
        if (self.current_x_position > self.boundary_parameter-0.0875) & (self.current_y_position < self.negative_boundary_parameter+0.0875):
            self.quadrant = 4
            self.positive_quadrant1_origin = False
            self.negative_quadrant1_origin = False
            self.positive_quadrant2_origin = False
            self.negative_quadrant2_origin = True
            self.positive_quadrant3_origin = False
            self.negative_quadrant3_origin = False
            self.positive_quadrant4_origin = True
            self.negative_quadrant4_origin = False
            if ((self.current_z_orientation < -0.01) & (self.current_z_orientation > -0.697))or((self.current_z_orientation > 0.01) & (self.current_z_orientation < 0.697))or((self.current_z_orientation > 0.72) & (self.current_z_orientation < 0.99))or((self.current_z_orientation < -0.72) & (self.current_z_orientation > -0.99)):
                self.move_crossline = True
            elif ((self.current_z_orientation < 0.01) & (self.current_z_orientation > -0.01))or((self.current_z_orientation < -0.99) & (self.current_z_orientation > -1.0))or((self.current_z_orientation < -0.697) & (self.current_z_orientation > -0.72))or((self.current_z_orientation > 0.697) & (self.current_z_orientation < 0.72)):
                self.move_crossline = False
        # located in XY axis ((x<0.0125)&(x>-0.0125)) &((y<0.0125) & (y>-0.0125)) which means the origin define box size is 0.0125x0.0125
        # which means in origin
        if ((self.current_x_position < self.boundary_parameter-0.09) & ((self.current_x_position > self.negative_boundary_parameter+0.09))) & ((self.current_y_position < self.boundary_parameter-0.09) & (self.current_y_position > self.negative_boundary_parameter+0.09)):
            self.quadrant = 0
            self.leave_origin = False
            self.positive_x_origin = False
            self.positive_y_origin = False
            self.negative_x_origin = False
            self.negative_y_origin = False
            self.positive_quadrant1_origin = False
            self.negative_quadrant1_origin = False
            self.positive_quadrant2_origin = False
            self.negative_quadrant2_origin = False
            self.positive_quadrant3_origin = False
            self.negative_quadrant3_origin = False
            self.positive_quadrant4_origin = False
            self.negative_quadrant4_origin = False
            self.leave_origin_orientation_z = 0.0
            if ((self.current_z_orientation < -0.01) & (self.current_z_orientation > -0.697))or((self.current_z_orientation > 0.01) & (self.current_z_orientation < 0.697))or((self.current_z_orientation > 0.72) & (self.current_z_orientation < 0.99))or((self.current_z_orientation < -0.72) & (self.current_z_orientation > -0.99)):
                self.move_crossline = True
            elif ((self.current_z_orientation < 0.01) & (self.current_z_orientation > -0.01))or((self.current_z_orientation < -0.99) & (self.current_z_orientation > -1.0))or((self.current_z_orientation < -0.697) & (self.current_z_orientation > -0.72))or((self.current_z_orientation > 0.697) & (self.current_z_orientation < 0.72)):
                self.move_crossline = False

    def turn_right_function(self):
        if (self.over_half == 1)&(self.current_z_orientation > -0.999):
            self.data_odom_right = ModelState()
            self.data_odom_right.model_name = 'robot'

            self.moved_radian = abs(self.saved_moved_radian)+abs(self.abs_moved_radian)
            self.moved_radian = -self.moved_radian
            self.saved_moved_radian = self.moved_radian

            # pose
            self.data_odom_right.pose.position.x = self.current_x_position                  # change
            self.data_odom_right.pose.position.y = self.current_y_position                # change
            self.data_odom_right.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_right.pose.orientation.x = 0.0               # unchange
            self.data_odom_right.pose.orientation.y = 0.0               # unchange
            self.data_odom_right.pose.orientation.z = self.moved_radian            # change
            self.data_odom_right.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_right)
            self.final_z_orientation = self.data_odom_right.pose.orientation.z
            print (self.moved_radian)
            print (self.saved_moved_radian)
            print "The final x-axis position (cm) is : "+ str(self.distance_from_x_origin)        # unchange
            print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # unchange
            print "The final z-axis orientation (clockwise) is : "+ str(self.final_z_orientation)        # change
            print ""


        if self.over_half == 0:
            self.data_odom_right = ModelState()
            self.data_odom_right.model_name = 'robot'

            self.moved_radian = self.saved_moved_radian-self.abs_moved_radian
            self.saved_moved_radian = self.moved_radian

            # pose
            self.data_odom_right.pose.position.x = self.current_x_position                  # change
            self.data_odom_right.pose.position.y = self.current_y_position                # change
            self.data_odom_right.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_right.pose.orientation.x = 0.0               # unchange
            self.data_odom_right.pose.orientation.y = 0.0               # unchange
            self.data_odom_right.pose.orientation.z = self.moved_radian            # change
            self.data_odom_right.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_right)
            self.final_z_orientation = self.data_odom_right.pose.orientation.z
            print (self.moved_radian)
            print (self.saved_moved_radian)
            print "The final x-axis position (cm) is : "+ str(self.distance_from_x_origin)        # unchange
            print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # unchange
            print "The final z-axis orientation (clockwise) is : "+ str(self.final_z_orientation)        # change
            print ""

    def turn_left_function(self):
        if (self.over_half == 1)&(self.current_z_orientation < 0.999):
            self.data_odom_right = ModelState()
            self.data_odom_right.model_name = 'robot'

            self.moved_radian = abs(self.saved_moved_radian)+abs(self.abs_moved_radian)
            self.saved_moved_radian = self.moved_radian

            # pose
            self.data_odom_right.pose.position.x = self.current_x_position                  # change
            self.data_odom_right.pose.position.y = self.current_y_position                # change
            self.data_odom_right.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_right.pose.orientation.x = 0.0               # unchange
            self.data_odom_right.pose.orientation.y = 0.0               # unchange
            self.data_odom_right.pose.orientation.z = self.moved_radian            # change
            self.data_odom_right.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_right)
            self.final_z_orientation = self.data_odom_right.pose.orientation.z
            print (self.moved_radian)
            print (self.saved_moved_radian)
            print "The final x-axis position (cm) is : "+ str(self.distance_from_x_origin)        # unchange
            print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # unchange
            print "The final z-axis orientation (anticlockwise) is : "+ str(self.final_z_orientation)        # change
            print ""


        if self.over_half == 0:
            self.data_odom_right = ModelState()
            self.data_odom_right.model_name = 'robot'

            self.moved_radian = self.saved_moved_radian+self.abs_moved_radian
            self.saved_moved_radian = self.moved_radian

            # pose
            self.data_odom_right.pose.position.x = self.current_x_position                  # change
            self.data_odom_right.pose.position.y = self.current_y_position                # change
            self.data_odom_right.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_right.pose.orientation.x = 0.0               # unchange
            self.data_odom_right.pose.orientation.y = 0.0               # unchange
            self.data_odom_right.pose.orientation.z = self.moved_radian            # change
            self.data_odom_right.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_right)
            self.final_z_orientation = self.data_odom_right.pose.orientation.z
            print (self.moved_radian)
            print (self.saved_moved_radian)
            print "The final x-axis position (cm) is : "+ str(self.distance_from_x_origin)        # unchange
            print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # unchange
            print "The final z-axis orientation (anticlockwise) is : "+ str(self.final_z_orientation)        # change
            print ""

    def located_in_origin(self):
        self.data_odom_origin = ModelState()
        self.data_odom_origin.model_name = 'robot'

        # stay in origin

        # facing positive y-axis (forward)
        if ((self.move_crossline == False)&(self.current_z_orientation < 0.01) & (self.current_z_orientation > -0.01) & (self.quadrant == 0) & (self.leave_origin == False)):
                self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

                self.data_odom_origin.pose.position.x = self.current_x_position       # unchange
                self.data_odom_origin.pose.position.y = (self.distance_from_y_origin/100.0)     # change
                self.data_odom_origin.pose.position.z = 0.05               # unchange
                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = self.current_z_orientation            # unchange
                self.data_odom_origin.pose.orientation.w = 1.0            # unchange
                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)
                self.final_x_position = self.current_x_position

                print "The final x-axis position (cm) is : "+ str(self.final_x_position)        # unchange
                print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # change
                print ""
                # move to positive y and leave the origin
                if self.current_y_position > self.boundary_parameter:
                    self.leave_origin = True
                    self.positive_y_origin = True
                    self.positive_x_origin = False
                    self.negative_y_origin = False
                    self.negative_x_origin = False

        # facing negative y-axis (backward)
        if ((self.move_crossline == False)&(self.current_z_orientation < -0.99) & (self.current_z_orientation > -1.0) & (self.quadrant == 0) & (self.leave_origin == False)):
                if (self.move_forward == True)&(self.move_backward == False):
                    self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)
                elif (self.move_backward == True)&(self.move_forward == False):
                    self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

                self.data_odom_origin.pose.position.x = self.current_x_position       # unchange
                self.data_odom_origin.pose.position.y = self.distance_from_y_origin/100.0     # change
                self.data_odom_origin.pose.position.z = 0.05               # unchange
                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = -999999999999999999999999999999999999999999999999999.0            # unchange
                self.data_odom_origin.pose.orientation.w = 1.0            # unchange
                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)
                self.final_x_position = self.current_x_position

                print "The final x-axis position (cm) is : "+ str(self.final_x_position)        # unchange
                print "The final y-axis position (cm) is : "+ str(self.distance_from_y_origin)        # change
                print ""
                # move to positive y and leave the origin
                if abs(self.current_y_position) > abs(self.boundary_parameter):
                    self.leave_origin = True
                    self.negative_y_origin = True
                    self.positive_y_origin = False
                    self.positive_x_origin = False
                    self.negative_x_origin = False

        # facing positive x-axis (right)
        if ((self.move_crossline == False)&(self.current_z_orientation < -0.697) & (self.current_z_orientation > -0.72) & (self.quadrant == 0) & (self.leave_origin == False)):
                self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)

                self.data_odom_origin.pose.position.x = self.distance_from_x_origin/100.0       # change
                self.data_odom_origin.pose.position.y = self.current_y_position     # unchange
                self.data_odom_origin.pose.position.z = 0.05               # unchange

                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = -1.0            # unchange
                self.data_odom_origin.pose.orientation.w = 1.0            # unchange

                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)
                self.final_y_position = self.current_y_position

                print "The final x-axis position (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (cm) is : "+ str(self.final_y_position)        # unchange
                print ""
                # move to positive y and leave the origin
                if abs(self.current_x_position) > abs(self.boundary_parameter):
                    self.leave_origin = True
                    self.positive_x_origin = True
                    self.negative_x_origin = False
                    self.positive_y_origin = False
                    self.negative_y_origin = False

        # facing negative x-axis (left)
        if ((self.move_crossline == False)&(self.current_z_orientation > 0.697) & (self.current_z_orientation < 0.72) & (self.quadrant == 0) & (self.leave_origin == False)):
                if (self.move_forward == True)&(self.move_backward == False):
                    self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
                elif (self.move_backward == True)&(self.move_forward == False):
                    self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)

                self.data_odom_origin.pose.position.x = self.distance_from_x_origin/100.0       # change
                self.data_odom_origin.pose.position.y = self.current_y_position     # unchange
                self.data_odom_origin.pose.position.z = 0.05               # unchange

                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = 1.0            # unchange
                self.data_odom_origin.pose.orientation.w = 1.0            # unchange

                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)
                self.final_y_position = self.current_y_position

                print "The final x-axis position (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (cm) is : "+ str(self.final_y_position)        # unchange
                print ""
                # move to positive y and leave the origin
                if abs(self.current_x_position) > abs(self.boundary_parameter):
                    self.leave_origin = True
                    self.negative_x_origin = True
                    self.positive_x_origin = False
                    self.positive_y_origin = False
                    self.negative_y_origin = False

        # facing quadrant 1
        if ((self.move_crossline == True)&(self.current_z_orientation < -0.01) & (self.current_z_orientation > -0.697) & (self.quadrant == 0) & (self.leave_origin == False)):
                self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)
                self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

                self.data_odom_origin.pose.position.x = self.distance_from_x_origin/100.0       # change
                self.data_odom_origin.pose.position.y = self.distance_from_y_origin/100.0     # change
                self.data_odom_origin.pose.position.z = 0.0               # unchange

                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = self.current_z_orientation               # unchange

                if (self.turn_left == False)&(self.turn_right == False)&(self.move_forward == False)&(self.move_backward == False):
                    self.data_odom_origin.pose.orientation.z = abs(self.current_z_orientation)-0.00005      # add something to compensate the unknown changing while stop

                self.data_odom_origin.pose.orientation.w = 1.0            # unchange
                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)

                print "The final x-axis position (moving to quadrant 1) (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (moving to quadrant 1) (cm) is : "+ str(self.distance_from_y_origin)        # change
                print ""
                # move to quadrant 1 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.boundary_parameter-0.09)) & (self.current_y_position > (self.boundary_parameter-0.09)):
                    self.leave_origin = True
                    self.positive_quadrant1_origin = True
                    self.negative_quadrant1_origin = False
                    self.leave_origin_orientation_z = self.current_z_orientation
                # move to quadrant 3 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.negative_boundary_parameter+0.09)) & (self.current_y_position > (self.negative_boundary_parameter+0.09)):
                    self.leave_origin = True
                    self.positive_quadrant1_origin = False
                    self.negative_quadrant1_origin = True
                    self.leave_origin_orientation_z = self.current_z_orientation

        # facing quadrant 2
        if ((self.move_crossline == True)&(self.current_z_orientation > 0.01) & (self.current_z_orientation < 0.697) & (self.quadrant == 0) & (self.leave_origin == False)):
                self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
                self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

                self.data_odom_origin.pose.position.x = self.distance_from_x_origin/100.0       # change
                self.data_odom_origin.pose.position.y = self.distance_from_y_origin/100.0     # change
                self.data_odom_origin.pose.position.z = 0.0               # unchange

                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = self.current_z_orientation               # unchange

                if (self.turn_left == False)&(self.turn_right == False)&(self.move_forward == False)&(self.move_backward == False):
                    self.data_odom_origin.pose.orientation.z = abs(self.current_z_orientation)-0.00005      # add something to compensate the unknown changing while stop

                self.data_odom_origin.pose.orientation.w = 1.0            # unchange
                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)

                print "The final x-axis position (moving to quadrant 2) (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (moving to quadrant 2) (cm) is : "+ str(self.distance_from_y_origin)        # change
                print ""
                # move to quadrant 2 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.boundary_parameter-0.09)) & (self.current_y_position > (self.boundary_parameter-0.09)):
                    self.leave_origin = True
                    self.positive_quadrant2_origin = True
                    self.negative_quadrant2_origin = False
                    self.leave_origin_orientation_z = self.current_z_orientation
                # move to quadrant 4 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.negative_boundary_parameter+0.09)) & (self.current_y_position > (self.negative_boundary_parameter+0.09)):
                    self.leave_origin = True
                    self.positive_quadrant2_origin = False
                    self.negative_quadrant2_origin = True
                    self.leave_origin_orientation_z = self.current_z_orientation

        # facing quadrant 3
        if ((self.move_crossline == True)&(self.current_z_orientation > 0.72) & (self.current_z_orientation < 0.99) & (self.quadrant == 0) & (self.leave_origin == False)):
                self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
                self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

                self.data_odom_origin.pose.position.x = self.distance_from_x_origin/100.0       # change
                self.data_odom_origin.pose.position.y = self.distance_from_y_origin/100.0     # change
                self.data_odom_origin.pose.position.z = 0.0               # unchange

                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = self.current_z_orientation+1               # unchange

                if (self.turn_left == False)&(self.turn_right == False)&(self.move_forward == False)&(self.move_backward == False):
                    self.data_odom_origin.pose.orientation.z = abs(self.current_z_orientation)-0.00005      # add something to compensate the unknown changing while stop

                self.data_odom_origin.pose.orientation.w = 1.0            # unchange
                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)

                print "The final x-axis position (moving to quadrant 3) (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (moving to quadrant 3) (cm) is : "+ str(self.distance_from_y_origin)        # change
                print ""
                # move to quadrant 3 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.boundary_parameter-0.09)) & (self.current_y_position > (self.boundary_parameter-0.09)):
                    self.leave_origin = True
                    self.positive_quadrant3_origin = True
                    self.negative_quadrant3_origin = False
                    self.leave_origin_orientation_z = self.current_z_orientation
                # move to quadrant 1 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.negative_boundary_parameter+0.09)) & (self.current_y_position > (self.negative_boundary_parameter+0.09)):
                    self.leave_origin = True
                    self.positive_quadrant3_origin = False
                    self.negative_quadrant3_origin = True
                    self.leave_origin_orientation_z = self.current_z_orientation

        # facing quadrant 4
        if ((self.move_crossline == True)&(self.current_z_orientation < -0.72) & (self.current_z_orientation > -0.99) & (self.quadrant == 0) & (self.leave_origin == False)):
                self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)
                self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

                self.data_odom_origin.pose.position.x = self.distance_from_x_origin/100.0       # change
                self.data_odom_origin.pose.position.y = self.distance_from_y_origin/100.0     # change
                self.data_odom_origin.pose.position.z = 0.0               # unchange

                # orientation
                self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
                self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
                self.data_odom_origin.pose.orientation.z = self.current_z_orientation-1               # unchange

                if (self.turn_left == False)&(self.turn_right == False)&(self.move_forward == False)&(self.move_backward == False):
                    self.data_odom_origin.pose.orientation.z = abs(self.current_z_orientation)-0.00005      # add something to compensate the unknown changing while stop

                self.data_odom_origin.pose.orientation.w = 1.0            # unchange
                # update the setmodelstate 
                self.updateodom.publish(self.data_odom_origin)

                print "The final x-axis position (moving to quadrant 4) (cm) is : "+ str(self.distance_from_x_origin)        # change
                print "The final y-axis position (moving to quadrant 4) (cm) is : "+ str(self.distance_from_y_origin)        # change
                print ""
                # move to quadrant 4 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.boundary_parameter-0.09)) & (self.current_y_position > (self.boundary_parameter-0.09)):
                    self.leave_origin = True
                    self.positive_quadrant4_origin = True
                    self.negative_quadrant4_origin = False
                    self.leave_origin_orientation_z = self.current_z_orientation
                # move to quadrant 2 and leave the origin box (0.01x0.01)
                if (self.current_x_position > (self.negative_boundary_parameter+0.09)) & (self.current_y_position > (self.negative_boundary_parameter+0.09)):
                    self.leave_origin = True
                    self.positive_quadrant4_origin = False
                    self.negative_quadrant4_origin = True
                    self.leave_origin_orientation_z = self.current_z_orientation

    def moving_XY_axis_publish(self):
        self.data_odom_XY = ModelState()
        self.data_odom_XY.model_name = 'robot'
        # facing positive y-axis and move to positive y or facing positive y-axis and move to negative y
        if (self.positive_y_origin == True) :
            self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

            self.data_odom_XY.pose.position.x = self.current_x_position       # unchange
            self.data_odom_XY.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_XY.pose.position.z = 0.05               # unchange

            # orientation
            self.data_odom_XY.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_XY.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_XY.pose.orientation.z = abs(self.current_z_orientation)            # unchange
            self.data_odom_XY.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_XY)
            self.final_x_position = self.current_x_position

            print "The final x-axis position XY_axis (cm) is : "+ str(self.final_x_position)        # unchange
            print "The final y-axis position XY_axis (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""
        
        # facing negative y-axis and move to negative y or facing negative y-axis and move to positive y
        elif (self.negative_y_origin == True) :
            if (self.move_forward == True)&(self.move_backward == False):
                self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)
            elif (self.move_backward == True)&(self.move_forward == False):
                self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

            self.data_odom_XY.pose.position.x = self.current_x_position       # unchange
            self.data_odom_XY.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_XY.pose.position.z = 0.05               # unchange

            # orientation
            self.data_odom_XY.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_XY.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_XY.pose.orientation.z = -999999999999999999999999999999999999999999999999999.0            # unchange
            self.data_odom_XY.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_XY)
            self.final_x_position = self.current_x_position

            print "The final x-axis position XY_axis (cm) is : "+ str(self.final_x_position)        # unchange
            print "The final y-axis position XY_axis (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""
        
        # facing positive x-axis and move to positive x or facing positive x-axis and move to negative x
        elif (self.positive_x_origin == True) :
            self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)

            self.data_odom_XY.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_XY.pose.position.y = self.current_y_position     # unchange
            self.data_odom_XY.pose.position.z = 0.05               # unchange

            # orientation
            self.data_odom_XY.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_XY.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_XY.pose.orientation.z = -1.0            # unchange
            self.data_odom_XY.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_XY)
            self.final_y_position = self.current_y_position

            print "The final x-axis position XY_axis (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position XY_axis (cm) is : "+ str(self.final_y_position)        # unchange
            print ""

        # facing negative x-axis and move to negative x or facing negative x-axis and move to positive y
        elif (self.negative_x_origin == True) :
            if (self.move_forward == True)&(self.move_backward == False):
                self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
            elif (self.move_backward == True)&(self.move_forward == False):
                self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)

            self.data_odom_origin.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_origin.pose.position.y = self.current_y_position     # unchange
            self.data_odom_origin.pose.position.z = 0.05               # unchange

            # orientation
            self.data_odom_origin.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_origin.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_origin.pose.orientation.z = 1.0            # unchange
            self.data_odom_origin.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_origin)
            self.final_y_position = self.current_y_position

            print "The final x-axis position XY_axis (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position XY_axis (cm) is : "+ str(self.final_y_position)        # unchange
            print ""

    def moving_quadrant_publish(self):
        self.data_odom_quadrant = ModelState()
        self.data_odom_quadrant.model_name = 'robot'
        # facing quadrant 1 and move to quadrant 1
        if (self.positive_quadrant1_origin == True)&(self.negative_quadrant1_origin == False) :
            self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""

        # facing quadrant 1 and move to quadrant 3
        if (self.positive_quadrant1_origin == False)&(self.negative_quadrant1_origin == True) :
            self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""

        # facing quadrant 2 and move to quadrant 2
        if (self.positive_quadrant2_origin == True)&(self.negative_quadrant2_origin == False) :
            self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""

        # facing quadrant 2 and move to quadrant 4
        if (self.positive_quadrant2_origin == False)&(self.negative_quadrant2_origin == True) :
            self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""

        # facing quadrant 3 and move to quadrant 3
        if (self.positive_quadrant3_origin == True)&(self.negative_quadrant3_origin == False) :
            self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z+1            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""

        # facing quadrant 3 and move to quadrant 1
        if (self.positive_quadrant3_origin == False)&(self.negative_quadrant3_origin == True) :
            self.distance_from_x_origin = float(-self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z+1            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""

        # facing quadrant 4 and move to quadrant 4
        if (self.positive_quadrant4_origin == True)&(self.negative_quadrant4_origin == False) :
            self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z+1            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""

        # facing quadrant 4 and move to quadrant 2
        if (self.positive_quadrant4_origin == False)&(self.negative_quadrant4_origin == True) :
            self.distance_from_x_origin = float(self.moved_distance+self.saved_x_position)
            self.distance_from_y_origin = float(-self.moved_distance+self.saved_y_position)

            self.data_odom_quadrant.pose.position.x = self.distance_from_x_origin/100.0       # change
            self.data_odom_quadrant.pose.position.y = self.distance_from_y_origin/100.0     # change
            self.data_odom_quadrant.pose.position.z = 0.0               # unchange

            # orientation
            self.data_odom_quadrant.pose.orientation.x = self.current_x_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.y = self.current_y_orientation               # unchange
            self.data_odom_quadrant.pose.orientation.z = self.leave_origin_orientation_z-1            # unchange
            self.data_odom_quadrant.pose.orientation.w = 1.0            # unchange

            # update the setmodelstate 
            self.updateodom.publish(self.data_odom_quadrant)

            print "The final x-axis position quadrant1 (cm) is : "+ str(self.distance_from_x_origin)        # change
            print "The final y-axis position quadrant1 (cm) is : "+ str(self.distance_from_y_origin)        # change
            print ""


if __name__ == "__main__":
        try:
            odom = OdomEncoder()
            while not rospy.is_shutdown():
                odom.update()
                rospy.sleep(rospy.Duration(0.1))
                odom.pose()
                odom.calculation()
                odom.define_quadrant()
                print "----------------------------"
                print "current_x_position :"
                print (odom.current_x_position)
                print "current_y_position :"
                print (odom.current_y_position)
                print "current_z_orientation :"
                print (odom.current_z_orientation)
                print "boundary_parameter :"
                print (odom.boundary_parameter-0.0875)
                print "quadrant :"
                print (odom.quadrant)
                print "over_half :"
                print (odom.over_half)
                print "leave_origin :"
                print (odom.leave_origin)
                print "turn_left :"
                print (odom.turn_left)
                print "turn_right :"
                print (odom.turn_right)
                print "move_crossline :"
                print (odom.move_crossline)
                print "----------------------------"
                # Turn right wherever it stay
                if ((odom.quadrant == 0)or(odom.quadrant == 1)or(odom.quadrant == 2)or(odom.quadrant == 3)or(odom.quadrant == 4)) & ((odom.leave_origin == False)or(odom.leave_origin == True))&(odom.turn_left == False)&(odom.turn_right == True):
                    odom.turn_right_function()
                # Turn left wherever it stay
                if ((odom.quadrant == 0)or(odom.quadrant == 1)or(odom.quadrant == 2)or(odom.quadrant == 3)or(odom.quadrant == 4)) & ((odom.leave_origin == False)or(odom.leave_origin == True))&(odom.turn_left == True)&(odom.turn_right == False):
                    odom.turn_left_function()
                # located in origin
                if (odom.quadrant == 0) & (odom.leave_origin == False)&(odom.turn_left == False)&(odom.turn_right == False)&(((odom.current_left>odom.saved_left)&(odom.current_right>odom.saved_right))or((odom.saved_left>odom.current_left)&(odom.saved_right>odom.current_right))):
                    odom.located_in_origin()
                # located in quadrant 1
                #if (odom.quadrant == 1) & (odom.positive_quadrant1_origin == True)&(odom.negative_quadrant1_origin == False)&(odom.leave_origin == True)&(odom.turn_left == False)&(odom.turn_right == False)&(((odom.current_left>odom.saved_left)&(odom.current_right>odom.saved_right))or((odom.saved_left>odom.current_left)&(odom.saved_right>odom.current_right))):
                #    odom.located_in_quadrant1()
                if (odom.quadrant == 0) & (odom.leave_origin == True)&(odom.turn_left == False)&(odom.turn_right == False)&(((odom.current_left>odom.saved_left)&(odom.current_right>odom.saved_right))or((odom.saved_left>odom.current_left)&(odom.saved_right>odom.current_right))):
                    odom.moving_XY_axis_publish()
                if ((odom.quadrant == 1)or(odom.quadrant == 2)or(odom.quadrant == 3)or(odom.quadrant == 4))& (odom.leave_origin == True)&(odom.turn_left == False)&(odom.turn_right == False)&(((odom.current_left>odom.saved_left)&(odom.current_right>odom.saved_right))or((odom.saved_left>odom.current_left)&(odom.saved_right>odom.current_right))):
                    odom.moving_quadrant_publish()


        except rospy.ROSInterruptException:
            pass