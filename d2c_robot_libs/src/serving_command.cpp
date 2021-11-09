#include "d2c_robot_libs/serving_command.hpp"

ServingCommand::ServingCommand()
{
    
    joint1 = 0.0;
    joint2 = 0.0;
    joint3 = 0.0;
}

ServingCommand::~ServingCommand()
{}


std::vector<float> ServingCommand::PickUpCommand()
{
    std::vector<float> target_joint_position_;
    joint2 = 1.52;
    joint3 = 1.52;
    return target_joint_position_;
}
void ServingCommand::PutDownCommand()
{
    joint2 = 1.52;
    joint3 = 1.52;
}
void ServingCommand::RobotArmStrechCommand()
{
    joint1 = 3.0; // TODO : set the length to spin rate

}
void ServingCommand::RobotArmPullCommand()
{
    joint1 = 0.0;
}