#-------------------------------------#
#build the yolo class
#-------------------------------------#
import colorsys
import os
import time

import numpy as np
import torch
import torch.nn as nn
from PIL import Image, ImageDraw, ImageFont

# custom package
from yolo4_tiny import YoloBody
from utils import (DecodeBox, letterbox_image, non_max_suppression, yolo_correct_boxes)

# ros package
import rospy
from std_msgs.msg import Int32
#--------------------------------------------#
# Before using the custom model to perform prediction, it required to modify 3 parameter
# model_path, classes_path and phi need to modify
# if the shape is not match, please reminder the modification for model_path, classes_path and phi parameter
#--------------------------------------------#
class YOLO(object):
    _defaults = {
        "model_path"        : 'model_data/Epoch96-Total_Loss0.2573-Val_Loss0.1988.pth',
        "anchors_path"      : 'model_data/yolo_anchors.txt',
        "classes_path"      : 'model_data/voc_classes.txt',
        #-------------------------------#
        #    The attention type
        #   phi = 0  means not using attention
        #   phi = 1 for SE
        #   phi = 2 for CBAM
        #   phi = 3 for ECA
        #-------------------------------#
        "phi"               : 0,
        "model_image_size"  : (416, 416, 3),
        "confidence"        : 0.5,
        "iou"               : 0.3, 
        "cuda"              : True,
        #---------------------------------------------------------------------#
        #   letterbox image refer to whether using letterbox_image to resize the input image without distortion
        #   after few trail, it founds that not using letterbox_image to resize the input image have better performance
        #---------------------------------------------------------------------#
        "letterbox_image"   : False,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    #---------------------------------------------------#
    #  init the YOLO
    #---------------------------------------------------#
    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.generate()

    #---------------------------------------------------#
    #   get all the class
    #---------------------------------------------------#
    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names
    
    #---------------------------------------------------#
    #   get all the anchors
    #---------------------------------------------------#
    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape([-1, 3, 2])

    #---------------------------------------------------#
    #   generate the model
    #---------------------------------------------------#
    def generate(self):
        #---------------------------------------------------#
        #   build the yolov4-tiny model
        #---------------------------------------------------#
        self.net = YoloBody(len(self.anchors[0]), len(self.class_names), self.phi).eval()

        #---------------------------------------------------#
        #   load the yolov4-tiny model weight
        #---------------------------------------------------#
        print('Loading weights into state dict...')
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        state_dict = torch.load(self.model_path, map_location=device)
        self.net.load_state_dict(state_dict)
        print('Finished!')
        
        if self.cuda:
            self.net = nn.DataParallel(self.net)
            self.net = self.net.cuda()
    
        #---------------------------------------------------#
        #   build the decodebox tool for feature layer
        #---------------------------------------------------#
        self.yolo_decodes = []
        self.anchors_mask = [[3,4,5],[1,2,3]]
        for i in range(2):
            self.yolo_decodes.append(DecodeBox(np.reshape(self.anchors,[-1,2])[self.anchors_mask[i]], len(self.class_names),  (self.model_image_size[1], self.model_image_size[0])))

        print('{} model, anchors, and classes loaded.'.format(self.model_path))
        # setup different colour for the bounding box
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
    
    #---------------------------------------------------#
    #   detect the image
    #---------------------------------------------------#
    def detect_image(self, image):
        global turn_left
        global turn_right
        global detection_score
        global detection_top_ymin
        global detection_top_xmin
        global detection_top_ymax
        global detection_top_xmax
        #---------------------------------------------------------#
        #  convert the image to RGB format to avoid the gray scale format cause error during detection
        #---------------------------------------------------------#
        image = image.convert('RGB')

        image_shape = np.array(np.shape(image)[0:2])
        #---------------------------------------------------------#
        #    add gray bars to achieve undistorted resize
        #    it can also directly resize and implement in classification
        #---------------------------------------------------------#
        if self.letterbox_image:
            crop_img = np.array(letterbox_image(image, (self.model_image_size[1],self.model_image_size[0])))
        else:
            crop_img = image.resize((self.model_image_size[1],self.model_image_size[0]), Image.BICUBIC)
        photo = np.array(crop_img,dtype = np.float32) / 255.0
        photo = np.transpose(photo, (2, 0, 1))
        #---------------------------------------------------------#
        #    add the batch_size dimension
        #---------------------------------------------------------#
        images = [photo]

        with torch.no_grad():
            images = torch.from_numpy(np.asarray(images))
            if self.cuda:
                images = images.cuda()

            #---------------------------------------------------------#
            #  load the image into the network to perform prediction
            #---------------------------------------------------------#
            outputs = self.net(images)
            output_list = []
            for i in range(2):
                output_list.append(self.yolo_decodes[i](outputs[i]))

            #---------------------------------------------------------#
            #  stack the predicted bounding box and perform non_max_suppression
            #---------------------------------------------------------#
            output = torch.cat(output_list, 1)
            batch_detections = non_max_suppression(output, len(self.class_names),
                                                    conf_thres=self.confidence,
                                                    nms_thres=self.iou)
        
            #---------------------------------------------------------#
            #    if no object detected, it will return the original image
            #---------------------------------------------------------#
            try:
                batch_detections = batch_detections[0].cpu().numpy()
            except:
                turn_right = False
                turn_left = False
                detection_score=0
                detection_top_ymin=0
                detection_top_xmin=0
                detection_top_ymax=0
                detection_top_xmax=0
                return image
            
            #---------------------------------------------------------#
            #  screening of prediction boxes score
            #---------------------------------------------------------#
            top_index = batch_detections[:,4] * batch_detections[:,5] > self.confidence
            top_conf = batch_detections[top_index,4]*batch_detections[top_index,5]
            top_label = np.array(batch_detections[top_index,-1],np.int32)
            top_bboxes = np.array(batch_detections[top_index,:4])
            top_xmin, top_ymin, top_xmax, top_ymax = np.expand_dims(top_bboxes[:,0],-1),np.expand_dims(top_bboxes[:,1],-1),np.expand_dims(top_bboxes[:,2],-1),np.expand_dims(top_bboxes[:,3],-1)

            #-----------------------------------------------------------------#
            #    As letterbox_image parameter will ad the gray line before load the image into the network
            #    therefore, the top_bboxes will included the gray line
            #    it required to remove the gray line before having other process
            #-----------------------------------------------------------------#
            if self.letterbox_image:
                boxes = yolo_correct_boxes(top_ymin,top_xmin,top_ymax,top_xmax,np.array([self.model_image_size[0],self.model_image_size[1]]),image_shape)
            else:
                top_xmin = top_xmin / self.model_image_size[1] * image_shape[1]
                top_ymin = top_ymin / self.model_image_size[0] * image_shape[0]
                top_xmax = top_xmax / self.model_image_size[1] * image_shape[1]
                top_ymax = top_ymax / self.model_image_size[0] * image_shape[0]
                boxes = np.concatenate([top_ymin,top_xmin,top_ymax,top_xmax], axis=-1)

        font = ImageFont.truetype(font='model_data/simhei.ttf',size=np.floor(3e-2 * np.shape(image)[1] + 0.5).astype('int32'))

        thickness = max((np.shape(image)[0] + np.shape(image)[1]) // self.model_image_size[0], 1)

        """
        if there are two object class in your dataset and you only want to get one object class in a particular time
        you should use the following function to remove the lower score detection
        """

        if len(top_conf) == 1:
            if top_label[0] == 1: # turn right detected
                turn_right = True
                turn_left = False
                detection_score=[top_conf[0]] # get the mark
                detection_top_ymin=[boxes[0][0]]
                detection_top_xmin=[boxes[0][1]]
                detection_top_ymax=[boxes[0][2]]
                detection_top_xmax=[boxes[0][3]]

            if top_label[0] == 0: # turn left detected
                turn_left = True
                turn_right = False
                detection_score=[top_conf[0]] # get the mark
                detection_top_ymin=[boxes[0][0]]
                detection_top_xmin=[boxes[0][1]]
                detection_top_ymax=[boxes[0][2]]
                detection_top_xmax=[boxes[0][3]]

        elif len(top_conf) == 2:    # more than one object is detected
            #compare two object score
            if top_conf[0] > top_conf[1]:    # left score larger than right score
                detection_score=[top_conf[0]]
                top_label=[top_label[0]]
                turn_left = True
                turn_right = False
                detection_top_ymin=[boxes[0][0]]
                detection_top_xmin=[boxes[0][1]]
                detection_top_ymax=[boxes[0][2]]
                detection_top_xmax=[boxes[0][3]]
                print ("{} is detected : {}".format("turn_left",score))

            elif top_conf[1] > top_conf[0]:    # right score larger than left score
                detection_score=[top_conf[1]]
                top_label=[top_label[1]]
                turn_right = True
                turn_left = False
                detection_top_ymin=[boxes[0][0]]
                detection_top_xmin=[boxes[0][1]]
                detection_top_ymax=[boxes[0][2]]
                detection_top_xmax=[boxes[0][3]]
                print ("{} is detected : {}".format("turn_right",score))


        for i, c in enumerate(top_label):
            predicted_class = self.class_names[c]
            score = top_conf[i]

            top, left, bottom, right = boxes[i]
            top = top - 5
            left = left - 5
            bottom = bottom + 5
            right = right + 5

            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(np.shape(image)[0], np.floor(bottom + 0.5).astype('int32'))
            right = min(np.shape(image)[1], np.floor(right + 0.5).astype('int32'))


            # draw the bounding box
            label = '{} :{:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)
            print(label, top, left, bottom, right)
            
            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            for i in range(thickness):
                draw.rectangle(
                    [left + i, top + i, right - i, bottom - i],
                    outline=self.colors[self.class_names.index(predicted_class)])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=self.colors[self.class_names.index(predicted_class)])
            draw.text(text_origin, str(label), fill=(0, 0, 0), font=font)
            del draw

        return image


    def get_FPS(self, image, test_interval):
        image_shape = np.array(np.shape(image)[0:2])
        if self.letterbox_image:
            crop_img = np.array(letterbox_image(image, (self.model_image_size[1],self.model_image_size[0])))
        else:
            crop_img = image.resize((self.model_image_size[1],self.model_image_size[0]), Image.BICUBIC)
        photo = np.array(crop_img,dtype = np.float32) / 255.0
        photo = np.transpose(photo, (2, 0, 1))
        images = [photo]

        with torch.no_grad():
            images = torch.from_numpy(np.asarray(images))
            if self.cuda:
                images = images.cuda()

            outputs = self.net(images)
            output_list = []
            for i in range(2):
                output_list.append(self.yolo_decodes[i](outputs[i]))

            output = torch.cat(output_list, 1)
            batch_detections = non_max_suppression(output, len(self.class_names), conf_thres=self.confidence, nms_thres=self.iou)
            try:
                batch_detections = batch_detections[0].cpu().numpy()
                top_index = batch_detections[:,4] * batch_detections[:,5] > self.confidence
                top_conf = batch_detections[top_index,4]*batch_detections[top_index,5]
                top_label = np.array(batch_detections[top_index,-1],np.int32)
                top_bboxes = np.array(batch_detections[top_index,:4])
                top_xmin, top_ymin, top_xmax, top_ymax = np.expand_dims(top_bboxes[:,0],-1),np.expand_dims(top_bboxes[:,1],-1),np.expand_dims(top_bboxes[:,2],-1),np.expand_dims(top_bboxes[:,3],-1)

                if self.letterbox_image:
                    boxes = yolo_correct_boxes(top_ymin,top_xmin,top_ymax,top_xmax,np.array([self.model_image_size[0],self.model_image_size[1]]),image_shape)
                else:
                    top_xmin = top_xmin / self.model_image_size[1] * image_shape[1]
                    top_ymin = top_ymin / self.model_image_size[0] * image_shape[0]
                    top_xmax = top_xmax / self.model_image_size[1] * image_shape[1]
                    top_ymax = top_ymax / self.model_image_size[0] * image_shape[0]
                    boxes = np.concatenate([top_ymin,top_xmin,top_ymax,top_xmax], axis=-1)
            except:
                pass

        t1 = time.time()
        for _ in range(test_interval):
            with torch.no_grad():
                outputs = self.net(images)
                output_list = []
                for i in range(2):
                    output_list.append(self.yolo_decodes[i](outputs[i]))

                output = torch.cat(output_list, 1)
                batch_detections = non_max_suppression(output, len(self.class_names), conf_thres=self.confidence, nms_thres=self.iou)
                try:
                    batch_detections = batch_detections[0].cpu().numpy()
                    top_index = batch_detections[:,4] * batch_detections[:,5] > self.confidence
                    top_conf = batch_detections[top_index,4]*batch_detections[top_index,5]
                    top_label = np.array(batch_detections[top_index,-1],np.int32)
                    top_bboxes = np.array(batch_detections[top_index,:4])
                    top_xmin, top_ymin, top_xmax, top_ymax = np.expand_dims(top_bboxes[:,0],-1),np.expand_dims(top_bboxes[:,1],-1),np.expand_dims(top_bboxes[:,2],-1),np.expand_dims(top_bboxes[:,3],-1)

                    if self.letterbox_image:
                        boxes = yolo_correct_boxes(top_ymin,top_xmin,top_ymax,top_xmax,np.array([self.model_image_size[0],self.model_image_size[1]]),image_shape)
                    else:
                        top_xmin = top_xmin / self.model_image_size[1] * image_shape[1]
                        top_ymin = top_ymin / self.model_image_size[0] * image_shape[0]
                        top_xmax = top_xmax / self.model_image_size[1] * image_shape[1]
                        top_ymax = top_ymax / self.model_image_size[0] * image_shape[0]
                        boxes = np.concatenate([top_ymin,top_xmin,top_ymax,top_xmax], axis=-1)
                except:
                    pass
        t2 = time.time()
        tact_time = (t2 - t1) / test_interval
        return tact_time

    def detection_result(self):
        left=turn_left
        right=turn_right
        transfer_score=detection_score
        transfer_top_ymin=detection_top_ymin
        transfer_top_xmin=detection_top_xmin
        transfer_top_ymax=detection_top_ymax
        transfer_top_xmax=detection_top_xmax
        return left, right,transfer_score,transfer_top_ymin,transfer_top_xmin,transfer_top_ymax,transfer_top_xmax
