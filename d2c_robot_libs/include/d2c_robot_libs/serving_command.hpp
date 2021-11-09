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
        std::vector<float> PickUpCommand();
        void PutDownCommand();
        void RobotArmStrechCommand();
        void RobotArmPullCommand();

    private:
        int serving_command; 
        float joint1, joint2, joint3; //joint1 : screw , joint2 : left , joint3 :right
};


#endif 