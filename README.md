# 2021 AI Robot Hackathon in MAGOK

### 2021 마곡 AI 로봇 해커톤 팀 D2C
### Fully Automatic Serving Robot
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
3. Related Open Source
4. How to Start
5. Team Information


## 1. Project Introduction
Serving robots that are currently applied to most restaurants are half-automatic.
When serving, the robot brings the bowl to the side of the table, but the customer has to move the bowl to the table by himself.
The biggest problem is that you absolutely need staff to pick up the bowls.
So, restaurant operators who currently use serving robots on the market require additional staff even if they have robots.

#### These problems can be solved with Fully-Automatic Serving Robot.
## 2. Features

## 3. Related Open Source
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## 4. How to Start
#### < Quick start : rviz simulation >
<pre>
<code>
$ roslaunch d2c_robot_description display.launch
</code>
</pre>

![mail naver com](https://user-images.githubusercontent.com/19335771/142581467-6c7ccb58-d063-4bc8-b28f-a68cbcc55d91.jpeg)

#### < d2c robot control >
<pre>
<code>
$ roslaunch d2c_robot_libs d2c_robot.launch //keyboard control
$ roslaunch dynamixel_workbench_controller dynamixel_controller.launch 
$ roslaunch dynamixel_workbench_operator joint_operator.launch
</code>
</pre>

#### < YOLOv3-tiny boul detection >
<pre>
<code>
$ roslaunch darknet_ros darknet_ros.launch
$ roslaunch usb_cam usb_cam-test.launch
</code>
</pre>

## 5. Team Information
- Jung Hwan Seok : ROS, AI Developer
- Nam Yong Jae   : Hardware Developer

