### Base on Digital Twins IC382 from https://github.com/vincent51689453/Digital_Twins_IC382

### 1 Configure ROS master uri settings
### 1.1 Find out the host IP address
Type ```ifconifg``` in command line for both master and slave host
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/ifconfig.png)
### 1.2 Change the bashrc file configuration
Open new terminal and type ```sudo gedit .bashrc```in command line to open bashrc file
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/open_bashrc.png)

Demo picture for slave bashrc file:
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/slaver_bashrc.png)
* Add/Change the ROS_MASTER_URI with slave IP address such as```http://192.168.1.1:11311```
* Add/Change the ROS_IP with master IP address such as ```ROS_IP=192.168.1.2```

Demo picture for master bashrc file:
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/master_bashrc.png)
* Add/Change the ROS_MASTER_URI with master IP address such as```http://192.168.1.1:11311```
* Add/Change the ROS_IP with master IP address such as ```ROS_IP=192.168.1.1```

### 1.3 Change the host file configuration
Open new terminal and type following command to change host file setting
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/open_hosts.png)

Demo picture for slave host file:
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/slaver_host.png)
* Add/Change the slave IP address with slave host name such as ```192.168.1.1    slave_host_name```
* Add/Change the master IP address with master host name such as ```192.168.1.2    master_host_name```

Demo picture for master host file:
![image](https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/photo/master_host.png)
* Add/Change the master IP address with master host name such as ```192.168.1.2    master_host_name```
* Add/Change the slave IP address with slave host name such as ```192.168.1.1    slave_host_name```

### 1.4 Connect master and slave host with Lan connection and type ```source ~/.bashrc```

### 2 YOLO installation
### 2.1 Python2.7 Package preparation before using YOLOv4-tiny
Please review the requirement text file https://github.com/laitathei/Gazebo-rosserial-rescue-robot/blob/main/yolov4-tiny/YOLO_requirement.txt
Friendly reminder for using PIL python2.7 package, Please ```sudo apt-get install libjpeg8-dev zlib1g-dev libfreetype6-dev``` first otherwise the PIL will not function as normal
### 2.2 Download YOLO script from this repository
Type ```git clone https://github.com/laitathei/Gazebo-rosserial-rescue-robot``` to download the script
### 3 YOLO training
### 3.1 Prepare the dataset via labelImg
* YOLO detection requirement VOC format dataset. labelImg can generate two kind of dataset which are VOC format and YOLO format. 
* Please use the window OS to download labelImg because the Qt version have conflict with python2 and python3. 
* The XML files will mix up with JPEG files in the same directory. Please seperate into two holder which are Annotations and JPEGImages. 
* Annotations will store the XML files and JPEGImages will store the JPEG file
* For the tutorial of using labelImg, please refer to https://github.com/tzutalin/labelImg

### 3.2 Generate train.txt file with bounding box XY coordinate
Before running the python script, please change the path with your own path
1. Using ```voc2yolov4.py``` to shuffle the dataset into different data such as train, test, validation
2. Using ```voc_annotation.py``` to generate text file with with bounding box XY coordinate base on ImageSets text file information
If you have more than one object have been labelled, please add it into voc_annotation.py `classes = ["xx", "xx"]`
Also, add the class name into ```model_data/voc_classes.txt```with same order

The final dataset structure:
```
--workspace
          --VOCdevkit
                      --VOC2007
                                --Annotations
                                --ImageSets
                                            --Main
                                                   --test.txt
                                                   --train.txt
                                                   --trainval.txt
                                                   --val.txt
                                                   
                                --JPEGImages
```
### 3.3 Config hyperparameters (optional)
In train.py, you can change:
* cuda (True/False)
* normalize (True/False)
* input_shape
* mosaic (True/False)
* cosine_scheduler (True/False)
* label_smoothing
* learning rate
* batch_size
* Init_Epoch
* Freeze_Epoch

### 3.4 Training
* Type ```python2.7 train.py``` in command line
* The validation loss and total loss will shown on the command line
* Also, the final model will also shown the validation loss and total loss

### 4 YOLO Deployment
### 4.1 Get the class label, scores and boundary boxes coordinates
Class label have the constant order which means the class label generated from detection also have constant order
```
        global turn_left
        global turn_right
        global detection_score
        global detection_top_ymin
        global detection_top_xmin
        global detection_top_ymax
        global detection_top_xmax
        ......
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
            ......
    def detection_result(self):
        left=turn_left
        right=turn_right
        transfer_score=detection_score
        transfer_top_ymin=detection_top_ymin
        transfer_top_xmin=detection_top_xmin
        transfer_top_ymax=detection_top_ymax
        transfer_top_xmax=detection_top_xmax
        return left, right,transfer_score,transfer_top_ymin,transfer_top_xmin,transfer_top_ymax,transfer_top_xmax
```
Base on this two class example ```in yolo.py```, more than two class can be further develop
the detection_result function will store the required variable and just need to call it via ```yolo.detection_result```
