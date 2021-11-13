#include "d2c_robot_libs/serving_command.hpp"

ServingCommand::ServingCommand()
{}

ServingCommand::~ServingCommand()
{}


std::vector<float> ServingCommand::ReturnTargetPosition()
{
    std::vector<float> target_joint_position_ = {1.52, 0.0, 0.0, 0.0};
    //std::vector<double> object_position_;
    //object_position_ = SetTargetPosition();
    //target_joint_position_ = InverseKinematics(object_position_);

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
    //TODO :  
     

    return object_position;
}

std::vector<float> ServingCommand::Initialize()
{
    std::vector<float> initialize_position;
    initialize_position = {1.52, 0.0, 0.0, 0.0};
    return initialize_position;
}

std::vector<float> ServingCommand::Serving()
{
    std::vector<float> serving_position;
    return serving_position;
}

std::vector<float> ServingCommand::Cleaning(std::vector<float> object_position)
{
    std::vector<float> cleaning_position;
    return cleaning_position;
}