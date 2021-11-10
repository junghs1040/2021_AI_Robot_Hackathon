#include "d2c_robot_libs/serving_command.hpp"

ServingCommand::ServingCommand()
{}

ServingCommand::~ServingCommand()
{}


std::vector<float> ServingCommand::ReturnTargetPosition()
{
    std::vector<float> target_joint_position_;
    std::vector<float> object_position_;
    object_position_ = SetTargetPosition();
    target_joint_position_ = InverseKinematics(object_position_);

    return target_joint_position_;
}
std::vector<float> ServingCommand::InverseKinematics(std::vector<float> object_position)
{
    std::vector<float> target_joint_position;

    //TODO : calculation 

    return target_joint_position;
}
std::vector<float> ServingCommand::SetTargetPosition()
{
    std::vector<float> object_position;

    //TODO : get the information of object, and save into object_position variable
    //TODO :  - 

    return object_position;
}
void ServingCommand::RobotArmPullCommand()
{
    joint1 = 0.0;
}