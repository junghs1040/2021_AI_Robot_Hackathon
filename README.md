# 2021 AI Robot Hackathon in MAGOK
### Team D2C : Fully Automatic Serving Robot
( we won the Encouragement prize! )     
[![melodic-devel Status](https://github.com/ROBOTIS-GIT/dynamixel-workbench/workflows/melodic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/dynamixel-workbench/tree/melodic-devel)

Required version(tested):
- Ubuntu 18.04 (ROS Melodic)

Hardware Information:
- Jetson Nano Developer Kit
- OpenCR
- Dynamixel XL430-W250-T

## Table of Contents
1. Project Introduction
2. Features
3. Related Source
4. How to Start
5. Team Information


## 1. Project Introduction
Serving robots that are currently applied to most restaurants are half-automatic.
When serving, the robot brings the dishes to the side of the table, and the customer has to move the dishes to the table by himself.
or restaurant staff came to serve the dishes from robot to table.
The biggest problem is that you absolutely need staff to clean up the table.
So, restaurant operators who currently use serving robots nowdays require additional staff even if they have robots.

#### These problems can be solved with Fully-Automatic Serving Robot.
## 2. Features
d2c means that "delivery to(2) customer", we are developing the delivery and serving robot.

the robot uses the usb camera to detect the object(bowl).
we use YOLOv3-tiny to detect the object

If the robot detected the object(bowl) and its position. 
then the robot will calculate inverse kinematics of the manipulator and move the manipulator to the object.

## 3. Related Source
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [custom dynamixel_workbench](https://github.com/junghs1040/dynamixel-workbench)
- [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam)

## 4. How to Start
#### < Quick start : rviz simulation >
<pre>
<code>
$ roslaunch d2c_robot_description display.launch
</code>
</pre>

<center><img src="https://user-images.githubusercontent.com/19335771/142581467-6c7ccb58-d063-4bc8-b28f-a68cbcc55d91.jpeg" width="70%" height="70%"></center>

#### clone the related and essenial packages
<pre>
<code>
$ git clone https://github.com/leggedrobotics/darknet_ros.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/junghs1040/dynamixel-workbench.git
</code>
</pre>

#### < YOLOv3-tiny bowl detection >
roslaunch the package to turn on the YOLO object detection and use the usb camera
<pre>
<code>
$ roslaunch darknet_ros darknet_ros.launch
$ roslaunch usb_cam usb_cam-test.launch
</code>
</pre>

#### < d2c robot control >
roslaunch the package to control the robot and dynamixel
<pre>
<code>
$ roslaunch d2c_robot_libs d2c_robot.launch //keyboard control
$ roslaunch dynamixel_workbench_controller dynamixel_controller.launch 
$ roslaunch dynamixel_workbench_operator my_operator.launch
</code>
</pre>
you can control the robot by keyboard key (q,w,e) 
q: initialization (put manipulator aside to detect the object by camera)
w: serving motion
e: cleaning motion (when he object are detected)

### 1. Serving motion 
you can see that robot serving motion to pre defined coordinate
<center><img src="https://user-images.githubusercontent.com/19335771/156920406-d2a35c0c-3754-4a06-9626-7dd8cddfd078.PNG" width="70%" height="70%"></center>
Youtube link: https://www.youtube.com/watch?v=FxkeqOVxc3M

### 1. Cleaning motion 
first the robot detected the object(bowl), then he robot clean the bowl by moving to object's coordinate.
<center><img src="https://user-images.githubusercontent.com/19335771/156920596-83da5f2b-0f96-45dd-8288-b7c68ee72678.PNG" width="70%" height="70%"></center>
Youtube link: https://www.youtube.com/watch?v=MeFMGXxcLSg



## 5. Team Information
- Jung Hwan Seok (Hanyang univ. Mechanical Engineering) 
- Nam Yong Jae (Hanyang univ. Mechanical Engineering) 

#### Hackathon homepage : https://m-hackathon.tistory.com/

