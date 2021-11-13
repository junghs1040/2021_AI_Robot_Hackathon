#include "d2c_robot_libs/robot_node.hpp"

D2cControl::D2cControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv)
{
    double loop_rate = 100.0;
    
    serving_command_publisher = nh -> advertise<sensor_msgs::JointState>("joint_states", 1); // rviz simulation
    dynamixel_command_publiahser = nh -> advertise<d2c_robot_msgs::DynamixelCommand>("dynamixel_position_command", 1);
    
    object_position_subscriber = nh -> subscribe("d2c_robot_msg", 1000, &D2cControl::CommandmsgCallback, this);
    
    loop_timer = nh_priv->createTimer(ros::Duration(1/loop_rate), &D2cControl::controlLoop, this);
}

D2cControl::~D2cControl()
{}

void D2cControl::controlLoop(const ros::TimerEvent& event)
{
    target_joint_position = serving_command.ReturnTargetPosition();
    publishCommands(target_joint_position);
}

void D2cControl::CommandmsgCallback(const d2c_robot_msgs::D2cRobot::ConstPtr& msg)
{

    float motion_num = msg -> motion_command;
    ROS_INFO("Command info: %f", motion_num);

    if (motion_num = 0) // Initialize 
    {
        target_joint_position = serving_command.Initialize();
        d2c.motion = motion_num;
        d2c.position_info = something;
    }

    else if (motion_num = 1) // Serving 
    {
        target_joint_position = serving_command.Initialize();
        d2c.motion = motion_num;
        d2c.position_info = something;        
    }

    else if (motion_num = 2) // Cleaning
    {
        target_joint_position = serving_command.Initialize();
        d2c.motion = motion_num;
        d2c.position_info = something;
    }

    dynamixel_command_publiahser.publish(d2c);
}


void D2cControl::publishCommands(std::vector<float> target_joint_position)
{
    // joint_state publisher to RVIZ SIMULATION
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    
    for (int i=0; i<4; i++)
    {
        joint_state.name[i] = joint_name[i];
        joint_state.position[i] = target_joint_position[i];
    }

    //std_srvs::Triger trig;
    //dynamixel_command.DynamixelMsgCallback(trig.request, trig.response);
    serving_command_publisher.publish(joint_state);
}
// Code description
// Needed 1. Dynamixel control 2. Computer vision 3. Camera control 