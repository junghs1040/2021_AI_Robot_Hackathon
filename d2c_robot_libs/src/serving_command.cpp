#include "d2c_robot_libs/serving_command.hpp"

ServingCommand::ServingCommand()
{}

ServingCommand::~ServingCommand()
{}


std::vector<double> ServingCommand::ReturnTargetPosition()
{
    std::vector<double> target_joint_position_ = {1.52, 0.0, 0.0, 0.0};
    //std::vector<double> object_position_;
    //object_position_ = SetTargetPosition();
    //target_joint_position_ = InverseKinematics(object_position_);

    return target_joint_position_;
}
std::vector<double> ServingCommand::InverseKinematics(std::vector<double> object_position)
{
    std::vector<double> target_joint_position;

    //TODO : calculation 

    return target_joint_position;
}
std::vector<double> ServingCommand::SetTargetPosition()
{
    std::vector<double> object_position;

    //TODO : get the information of object, and save into object_position variable
    //TODO :  
     

    return object_position;
}
void ServingCommand::RobotArmPullCommand()
{
    joint1 = 0.0;
}