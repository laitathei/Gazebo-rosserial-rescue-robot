# coding=UTF-8
import colorsys
import os
import time

import numpy as np
import cv2
import cv_bridge
import torch
import torch.nn as nn
from PIL import ImageDraw, ImageFont
from PIL import Image as PIL_Image
from cv_bridge.boost.cv_bridge_boost import getCvType

# ros package
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
# custom package
from yolo4_tiny import YoloBody
from utils import (DecodeBox, letterbox_image, non_max_suppression, yolo_correct_boxes)
from yolo import YOLO


class object_detect:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node('object_detection')
        print ("init finish!")

    def image_callback(self, msg):
        # BEGIN BRIDGE
        self.img_detect = self.bridge.imgmsg_to_cv2(msg)

        # Resize the image
        self.img_detect = cv2.resize(self.img_detect,(720,480))

        print ("callback finish!")
        self.img = self.img_detect
        return self.img

    def result_callback(self, data):
        # get the detection result (turn left or turn right)
        self.result = str(data.data)
        print ("Detection result is : "+(self.result))

    def region_of_interest(self,img,vertices):
        self.mask = np.zeros_like(img) # convert the img ndarray to same size but all element is zero
        #Masking color
        self.match_mask_color = 255
        #Fill inside the polygon
        cv2.fillPoly(self.mask,vertices,self.match_mask_color)
        self.masked_image = cv2.bitwise_and(img,mask) # bitwise_and mean any element in the img ndarray will and gate with 0 separately
        return self.masked_image

    def image_detect(self):
        self.image_sub = rospy.Subscriber('/robot/camera1/image_raw', Image, self.image_callback)
        self.yolo = YOLO()

        self.fps = 0.0
        while(True):
            self.t1 = time.time()
            # get the frame
            # frame is ndarray
            # convert to image
            self.frame = PIL_Image.fromarray(np.uint8(self.img))
            # start to detect
            self.frame = np.array(self.yolo.detect_image(self.frame))
            # get the detection result
            self.left, self.right = self.yolo.detection_result()
            print ("Turn left : {}".format(self.left))
            print ("Turn right : {}".format(self.right))
            # RGBtoBGR
            self.frame = cv2.cvtColor(self.frame,cv2.COLOR_RGB2BGR)

            self.fps  = ( self.fps + (1./(time.time()-self.t1)) ) / 2
            print("fps= %.2f"%(self.fps))
            self.frame = cv2.putText(self.frame, "fps= %.2f"%(self.fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow("gazebo camera",self.frame)
            self.c= cv2.waitKey(1) & 0xff 

            if self.c==27:
                capture.release()
                break
        
        capture.release()
        out.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detect = object_detect()
    while not rospy.is_shutdown():
        detect.image_detect()
