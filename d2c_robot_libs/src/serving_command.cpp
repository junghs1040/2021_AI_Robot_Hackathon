#include "d2c_robot_libs/serving_command.hpp"

ServingCommand::ServingCommand()
{}

ServingCommand::~ServingCommand()
{}


std::vector<float> ServingCommand::ReturnTargetJointPosition()
{
    std::vector<float> target_joint_position_ = {1.52, 0.0, 0.0, 0.0};
    std::vector<int> object_position = TransformCoordinate();
    target_joint_position_ = InverseKinematics(object_position);
    return target_joint_position_;
}
std::vector<float> ServingCommand::InverseKinematics(std::vector<int> object_position)
{
    std::vector<float> target_joint_position;
    int x = object_position[0];
    int y = object_position[1];
    float object_x = (float)x;
    float object_y = (float)y;
    //TODO : calculate inverse kinematics

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

std::vector<int> ServingCommand::TransformCoordinate()
{
    std::vector<int> serving_position;
    int x = object_x_;
    int y = object_y_;
    // transform coordination 
    serving_position = {x,y};

    return serving_position;
}

std::vector<float> ServingCommand::Cleaning()
{
    std::vector<float> cleaning_position;
    return cleaning_position;
}