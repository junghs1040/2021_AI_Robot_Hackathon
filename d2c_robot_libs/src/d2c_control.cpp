#include "d2c_robot_libs/robot_node.hpp"

D2cControl::D2cControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv)
{
    double loop_rate = 100.0;
    
    serving_command_publisher = nh -> advertise<sensor_msgs::JointState>("joint_states", 1); // rviz simulation
    dynamixel_command_publisher = nh -> advertise<d2c_robot_msgs::DynamixelCommand>("dynamixel_workbench/dynamixel_position_command", 1);
    
    object_position_subscriber = nh -> subscribe("d2c_robot_msg", 1000, &D2cControl::CommandmsgCallback, this);
    client =nh -> serviceClient<std_srvs::Trigger>("/dynamixel_workbench/execution");
    
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

    //if (motion_num = 0.0) // Initialize 
    //{
    //    target_joint_position = serving_command.Initialize();
    //}

    //else if (motion_num = 1.0) // Serving 
    //{
    //    target_joint_position = serving_command.Initialize();        
    //}

    //else if (motion_num = 2.0) // Cleaning
    //{
    //    target_joint_position = serving_command.Initialize(); 
    //}

    d2c.motion = motion_num;
    d2c.position_info = something;
    ROS_INFO("Command info: %f", motion_num);
    dynamixel_command_publisher.publish(d2c);
    //client.call("{}");
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