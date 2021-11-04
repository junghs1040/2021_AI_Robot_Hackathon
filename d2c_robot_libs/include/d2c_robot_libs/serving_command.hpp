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
        void PickUpCommand();
        void PutDownCommand();
        void RobotArmStrechCommand();
        void RobotArmPullCommand();

    private:

};


#endif 