#ifndef SERVING_COMMAND_H_
#define SERVING_COMMAND_H_

#include "ros/ros.h"
#include "iostream"
#include "sensor_msgs/JointState.h"
#include "d2c_robot_libs/serving_command.hpp"

class ServingCommand
{
    public:
        ServingCommand();      
        ~ServingCommand();
        std::vector<std::vector<double>> ReturnTargetJointPosition();
        std::vector<double> InverseKinematics(std::vector<double> final_value);
        std::vector<std::vector<double>> SetTargetPosition();
        std::vector<float> Initialize();
        std::vector<int> TransformCoordinate();
        std::vector<float> Cleaning();
        
        int object_x_, object_y_;

    private:
        int serving_command; 
        std::vector<double> before_object_position_;
        std::vector<double> on_object_position_;
        std::vector<double> up_object_position_;
        float joint1, joint2, joint3, joint4; //joint1 : screw , joint2 : left , joint3 :right
};


#endif 