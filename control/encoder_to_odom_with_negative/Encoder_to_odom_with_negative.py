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
        self.updateodom = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.saved_left = 0.0
        self.saved_right = 0.0
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
        self.saved_left = self.current_left
        self.saved_right = self.current_right
        self.saved_x_position = self.current_x_position
        self.saved_y_position = self.current_y_position

    def pose(self):
        #print the message 
        print "The current LeftEncoder_value is : "+ str(self.current_left)
        print "The current RightEncoder_value is : "+ str(self.current_right)
        print "The previous x-axis position is : "+ str(self.current_x_position*100.0)
        print "The previous y-axis position is : "+ str(self.current_y_position*100.0)
        print "The previous z-axis position is : "+ str(self.current_z_position)
        print "The current z-axis orientation is : "+ str(self.final_z_position)
        print "The previous LeftEncoder_value is : "+ str(self.saved_left)
        print "The previous RightEncoder_value is : "+ str(self.saved_right)
        print "The saved x-axis distance is : "+ str(self.saved_x_position)
        print "The saved y-axis distance is : "+ str(self.saved_y_position)

    def calculation(self):
        # calculate the distance
        self.left_wheel_diff = float(self.current_left-self.saved_left)
        self.right_wheel_diff = float(self.current_right-self.saved_right)
        self.left_wheel_moved = (self.left_wheel_diff/self.encoder_one_rotation)*self.wheel_length
        self.right_wheel_moved = (self.right_wheel_diff/self.encoder_one_rotation)*self.wheel_length
        self.moved_distance =  (self.right_wheel_moved+self.left_wheel_moved)/2.0
        print "The moved distance (cm) is : "+ str(self.moved_distance)

    def publish(self):
        #self.updateodom.publish()

if __name__ == "__main__":
        try:
            odom = OdomEncoder()
            while not rospy.is_shutdown():
                odom.update()
                rospy.sleep(rospy.Duration(0.1))
                odom.pose()
                odom.calculation()
        except rospy.ROSInterruptException:
            pass