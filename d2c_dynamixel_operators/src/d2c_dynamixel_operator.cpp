#include "d2c_dynamixel_operators/d2c_joint_operator.h"
#include <iostream>
#include <vector>
#include <string>

JointOperator::JointOperator()
  :node_handle_(""),
   priv_node_handle_("~"),
   is_loop_(false)
{
  std::string yaml_file = node_handle_.param<std::string>("trajectory_info", "");
  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
  motion = 0;
  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);


  joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);
  dynamixel_command_subscriber_ = node_handle_.subscribe("dynamixel_position_command", 1000, &JointOperator::moveCommandMsgCallback, this);

  is_loop_ = priv_node_handle_.param<bool>("is_loop", "false");
}

JointOperator::~JointOperator()
{
}

bool JointOperator::getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
  // test with two joints
  std::vector<std::string> joint = {"joint1", "joint2"};
  for (uint8_t index = 0; index < joint.size(); index++)
  {
    jnt_tra_msg->joint_names.push_back(joint[index]);
  }

  std::vector<std::string> motion_name = motionNameChoice();
  std::vector<std::vector<double>> motion = motionChoice();
  std::vector<double> time_from_start = timeChoice();


  for (uint8_t index = 0; index < motion_name.size(); index++)
  {
    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

    for (uint8_t size = 0; size < joint.size(); size++)
    {
      if (joint.size() != 2)
      {
        ROS_ERROR("Please check motion step size. It must be equal to joint size");
        return 0;
      }

      jnt_tra_point.positions.push_back(motion[index][size]);

      ROS_INFO("motion_name : %s, step : %f", motion_name[index].c_str(), motion[index][size]);
    }

    jnt_tra_point.time_from_start.fromSec(time_from_start[index]);

    ROS_INFO("time_from_start : %f", time_from_start[index]);

    jnt_tra_msg->points.push_back(jnt_tra_point);
  }

  return true;
}

void JointOperator::moveCommandMsgCallback(const d2c_robot_msgs::DynamixelCommand::ConstPtr& msg)
{
  ROS_INFO("get!");
  joint_trajectory_pub_.publish(*jnt_tra_msg_);

}

std::vector<std::string> JointOperator::motionNameChoice()
{
  std::vector<std::string> initialize_motion = {"initialize_motion"};
  std::vector<std::string> serving_motion = {"motion1","motion2","motion3","motion4"};
  std::vector<std::string> cleaning_motion = {"motion1","motion2","motion3","motion4"};
  if (motion = 0)
  {
    return initialize_motion;
  }
  else if (motion = 1)
  {
    return serving_motion;
  }
  else if (motion = 2)
  {
    return cleaning_motion;
  }
}

std::vector<std::vector<double>> JointOperator::motionChoice()
{
  std::vector<std::vector<double>> initialize_motion = {{0.0, 0.0},{1.0, 1.0},{2.0, 2.0},{3.0, 3.0},{2.0, 2.0}};
  std::vector<std::vector<double>> serving_motion = {{0.0, 0.0},{1.0, 1.0},{2.0, 2.0},{3.0, 3.0},{2.0, 2.0}};
  std::vector<std::vector<double>> cleaning_motion = {{0.0, 0.0},{1.0, 1.0},{2.0, 2.0},{3.0, 3.0},{2.0, 2.0}};
  if (motion = 0)
  {
    return initialize_motion;
  }
  else if(motion = 1)
  {
    return serving_motion;
  }
  else if (motion = 2)
  {
    return cleaning_motion;
  }
}

std::vector<double> JointOperator::timeChoice()
{
  std::vector<double> initialize_time = {2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<double> serving_time = {2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<double> cleaning_time = {2.0, 3.0, 4.0, 5.0, 6.0};
  if (motion = 0)
  {
    return serving_time;
  }
  else if (motion = 1)
  {
    return serving_time;
  }
  else if (motion = 2)
  {
    return cleaning_time;
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_operator");
  JointOperator joint_operator;

  ROS_INFO("For now, you can use publish joint trajectory msgs by triggering service(/execution)");

  ros::spin();
  return 0;
}
