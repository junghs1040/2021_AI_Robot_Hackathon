#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "urdf/model.h"
#include "d2c_robot_description/IK_Solver.hpp"



int main(int argc, char** argv)
{
    ros::init(argc,argv,"kinematic");
    ros::NodeHandle nh;
    Robot robot;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(100); //10Hz = 0.1s loop


    std::string robot_description_string;
    nh.param("robot_description", robot_description_string, std::string());

    std::vector<std::string> joint_name = {"joint1", "joint2", "joint3", "joint4"};
    std::vector<double> joint = {0.0, 0.0, 0.0};
    std::vector<double> pose = {200.0, -100.0, 0.0}; // when test publish is fine - >//

    std::vector<double> joint_position = robot.Leg_IK(pose);

    sensor_msgs::JointState joint_state;
        // for odom pub
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok())
    {
  
        ROS_INFO("update joint state");

        joint_state.header.stamp = ros::Time::now();
        odom_trans.header.stamp = ros::Time::now();
        joint_state.name.resize(4); // joint_State의 name 3개로 확장
        joint_state.position.resize(4); // joint_state의 position 3개로 확장
        for (int i=0; i<4; i++)
        {
            joint_state.name[i] = joint_name[i];
        }

        joint_state.position[0]=joint_position[0];
        joint_state.position[1]=joint_position[1];
        joint_state.position[2]=joint_position[2];
        joint_state.position[3]=joint_position[3];
        //joint_state.position[0]=0.0;
        //joint_state.position[1]=0.0;
        //joint_state.position[2]=0.0;
        //joint_state.position[3]=0.0;

        joint_pub.publish(joint_state);

        ROS_INFO("%f,%f,%f,%f",joint_position[0],joint_position[1],joint_position[2],joint_position[3]);
        broadcaster.sendTransform(odom_trans);
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
        loop_rate.sleep();
    }

}
