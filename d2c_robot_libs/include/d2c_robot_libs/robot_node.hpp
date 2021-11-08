#ifndef ROBOT_NODE_H_
#define ROBOT_NODE_H_

#include "ros/ros.h"
#include "iostream"
#include "sensor_msgs/JointState.h"
#include "d2c_robot_libs/serving_command.hpp"
#include "d2c_robot_msgs/D2cRobot.h"


class D2cControl
{
    public:
        D2cControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv);
        ~D2cControl();
        void controlLoop(const ros::TimerEvent& event);
        void ObjectDetectmsgCallback(const d2c_robot_msgs::D2cRobot::ConstPtr& msg);
        void publishCommands(int order);

    private:
        ros::Timer loop_timer;
        ros::Publisher serving_command_publisher;
        ros::Subscriber object_position_subscriber;
        
        int control_command_; // 0: Initialize 1: Pick up , 2: Pull down, 3: Stretch Arm, 4:Pull Arm
        ServingCommand serving_command(int serve_com);

};


#endif