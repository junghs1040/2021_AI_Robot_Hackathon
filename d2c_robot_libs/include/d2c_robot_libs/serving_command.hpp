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
        std::vector<float> ReturnTargetJointPosition(int xmin, int ymin, int xmax, int ymax);
        std::vector<float> InverseKinematics(std::vector<float> object_position);
        std::vector<float> SetTargetPosition();
        std::vector<float> Initialize();
        std::vector<float> Serving();
        std::vector<float> Cleaning(std::vector<float> object_position);

    private:
        int serving_command; 
        
        float joint1, joint2, joint3, joint4; //joint1 : screw , joint2 : left , joint3 :right
};


#endif 