#ifndef ROBOT_NODE_H_
#define ROBOT_NODE_H_

#include "vector"
#include "string"
#include "ros/ros.h"
#include "iostream"
#include "sensor_msgs/JointState.h"
#include "d2c_robot_libs/serving_command.hpp"

#include "d2c_robot_msgs/D2cRobot.h"
#include "d2c_robot_msgs/DynamixelCommand.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <std_srvs/Trigger.h>


class D2cControl
{
    public:
        D2cControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv);
        ~D2cControl();
        void controlLoop(const ros::TimerEvent& event);
        void CommandmsgCallback(const d2c_robot_msgs::D2cRobot::ConstPtr& msg);
        void ObjectmsgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
        void publishCommands(std::vector<float> target_joint_position);
        

        
    private:
        ros::Timer loop_timer;
        ros::Publisher serving_command_publisher;
        ros::Publisher dynamixel_command_publisher;
        ros::ServiceServer dynamixel_command;
        ros::ServiceClient client;
        
        ros::Subscriber object_position_subscriber;
        ros::Subscriber control_keyboard_subscriber;
        std::vector<float> target_joint_position;
        int control_command_; // 0: Initialize 1: Pick up , 2: Pull down, 3: Stretch Arm, 4:Pull Arm
        float something;
        ServingCommand serving_command;
        sensor_msgs::JointState joint_state;
        d2c_robot_msgs::DynamixelCommand d2c;
        std::vector<std::string> joint_name = {"joint1","joint2","joint3","joint4"};
        int x_min, y_min, x_max, y_max;
        int object_x, object_y;
};


#endif