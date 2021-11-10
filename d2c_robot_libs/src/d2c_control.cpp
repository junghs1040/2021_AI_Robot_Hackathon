#include "d2c_robot_libs/robot_node.hpp"

D2cControl::D2cControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv)
{
    double loop_rate = 100.0;
    serving_command_publisher = nh->advertise<sensor_msgs::JointState>("joint_states", 1); // rviz simulation
     
    object_position_subscriber = nh ->subscribe("position_info_msg", 1000, &D2cControl::ObjectDetectmsgCallback, this);

    loop_timer = nh_priv->createTimer(ros::Duration(1/loop_rate), &D2cControl::controlLoop, this);
}

D2cControl::~D2cControl()
{}

void D2cControl::controlLoop(const ros::TimerEvent& event)
{
    std::vector<float> target_joint_position;
    
    target_joint_position = serving_command.ReturnTargetPosition();
    publishCommands(target_joint_position);
}

void D2cControl::ObjectDetectmsgCallback(const d2c_robot_msgs::D2cRobot::ConstPtr& msg)
{
    float x = msg->position[0];
    float y = msg->position[1];
    float z = msg->position[2];
    ROS_INFO("Position info : %f,%f,%f", x,y,z);
    std::vector<float> object_position = {x,y,z};
}

void D2cControl::publishCommands(std::vector<float> target_joint_position)
{
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4);
    joint_state.position.resize(4);

    for (int i=0; i<4; i++)
    {
        joint_state.name[i] = joint_name[i];
        joint_state.position[i]=target_joint_position[i];
    }

    serving_command_publisher.publish(joint_state);
}
// Code description
// Needed 1. Dynamixel control 2. Computer vision 3. Camera control 