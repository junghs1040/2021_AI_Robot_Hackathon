#include "d2c_robot_libs/robot_node.hpp"

D2cControl::D2cControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv)
{
    double loop_rate = 100.0;
    
    serving_command_publisher = nh -> advertise<sensor_msgs::JointState>("joint_states", 1); // rviz simulation
    dynamixel_command_publisher = nh -> advertise<d2c_robot_msgs::DynamixelCommand>("dynamixel_workbench/dynamixel_position_command", 1);
    
    control_keyboard_subscriber = nh -> subscribe("d2c_robot_msg", 1000, &D2cControl::CommandmsgCallback, this);
    object_position_subscriber = nh -> subscribe("darknet_ros/bounding_boxes", 1000, &D2cControl::ObjectmsgCallback, this);
    client =nh -> serviceClient<std_srvs::Trigger>("/dynamixel_workbench/execution");
    
    loop_timer = nh_priv->createTimer(ros::Duration(1/loop_rate), &D2cControl::controlLoop, this);
}

D2cControl::~D2cControl()
{}

void D2cControl::controlLoop(const ros::TimerEvent& event)
{
    //publishCommands(target_joint_position);
}

void D2cControl::CommandmsgCallback(const d2c_robot_msgs::D2cRobot::ConstPtr& msg)
{
    Eigen::Matrix3d m1, m2;
    Eigen::Vector3d v1(1,1,0);
    Eigen::Vector3d v2;
    m1 = serving_command.Rx(-3.14/2);
    m2 = serving_command.Ry(3.14/2);
   std::cout << m2*m1*v1 <<std::endl;
    float motion_num = msg -> motion_command;
    ROS_INFO("Command info: %f", motion_num);

    if (motion_num == 0.0) // Initialize 
    {
        //target_joint_position = serving_command.Initialize();
        //test : target_joint_position ={{1.1,1.1,1.1,1.1},{1.2,1.1,1.4,1.1},{1.1,1.1,1.1,1.1},{1.1,1.1,1.5,1.1}};
    }

    else if (motion_num == 1.0) // Serving 
    {
        //target_joint_position = serving_command.ReturnTargetJointPosition();        
    }
    
    else if (motion_num == 2.0) // Cleaning // TODO : bool && needed to confirm object position is getted
    {
        for(int i = 0; i < 2; i++)
        {
            d2c_robot_msgs::JointPosition position_info;

            for(int j = 0; j < 4; j++)
            {
                position_info.positions.push_back(target_joint_position[i][j]);
            }
            d2c.joint_position.push_back(position_info);
        }
        target_joint_position = serving_command.ReturnTargetJointPosition(); 
    }
    
    d2c.motion = motion_num;
    d2c.position_info = something;
    
    ROS_INFO("Command info: %f", motion_num);
    dynamixel_command_publisher.publish(d2c);

}

void D2cControl::ObjectmsgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    float probability = msg -> bounding_boxes[0].probability;
    int xmin = msg -> bounding_boxes[0].xmin;
    int ymin = msg -> bounding_boxes[0].ymin;
    int xmax = msg -> bounding_boxes[0].xmax;
    int ymax = msg -> bounding_boxes[0].ymax;
    std::string object_name = msg -> bounding_boxes[0].Class;
    

    if (object_name == "bowl")
    {
        ROS_INFO("%d, %d, %d, %d, %f, %s",xmin, ymin, xmax, ymax, probability, object_name.c_str());
        object_x = (xmin+xmax)/2;
        object_y = (ymin+ymax)/2;
        serving_command.object_x_ = object_x;
        serving_command.object_y_ = object_y;
        ROS_INFO("%d, %d", object_x, object_y);
    }
    //std::vector<float> dd = serving_command.ReturnTargetJointPosition(xmin, ymin, xmax, ymax);
    
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