### Base on Digital Twins IC382 from https://github.com/vincent51689453/Digital_Twins_IC382

### 1 Configure ROS master uri settings
### 1.1 Find out the host IP address
Type ```ifconifg``` in command line for both master and slave host
![image]()
### 1.2 Change the bashrc file configuration
Open new terminal and type ```sudo gedit .bashrc```in command line to open bashrc file
![image]()
### 1.3 Change the host file configuration
Open new terminal and type following command to change host file setting
![image]()
### 1.4 Connect master and slave host with Lan connection

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
1. Using ```voc2yolov4.py``` to shuffle the dataset into different data such as train, test, validation
2. Using ```voc_annotation.py``` to generate text file with with bounding box XY coordinate base on ImageSets text file information
