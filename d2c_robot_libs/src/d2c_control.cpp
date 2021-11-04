#include "d2c_robot_libs/robot_node.hpp"

D2cControl::D2cControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv)
{
    serving_command_publisher = nh->advertise<sensor_msgs::JointState>("joint_states", 1);

    object_position_subscriber = nh ->subscribe("position_info_msg", 1000, &D2cControl::msgCallback, this);
}

D2cControl::~D2cControl()
{}

void D2cControl::controlLoop(const ros::TimerEvent& event)
{
    

    publishCommands(control_order);
}

void D2cControl::msgCallback(const d2c_robot_msgs::D2cRobot::ConstPtr& msg)
{
    float x = msg->position[0];
    float y = msg->position[1];
    float z = msg->position[2];
    ROS_INFO("detected the object!, send the position info");
}

void D2cControl::publishCommands(int order)
{

}
// Code description
// Needed 1. Dynamixel control 2. Computer vision 3. Camera control 