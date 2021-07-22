#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge
import math
import sys
import geometry_msgs
#from cv_bridge.boost.cv_bridge_boost import getCvType
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs import msg
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int64, Header, String
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String, Duration, Header


class camera:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #use $rostopic list to find out the correct topic about images
    self.image_sub = rospy.Subscriber('/robot/camera1/image_raw', Image, self.image_callback)

  def image_callback(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg)
    #Convert BRG image to 
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite('/home/lai/catkin_ws/src/virtualrobotv2/vision/buffer.jpg',image)
    print("image is published")
    #cv2.imshow("camera_view(raw)", image)
    # END BRIDGE
    cv2.waitKey(3)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown:
            vistion_control = camera()
    except rospy.ROSInterruptException: pass


# END ALL