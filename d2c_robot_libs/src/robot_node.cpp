#include <vector>
#include <string>
#include "d2c_robot_libs/robot_node.hpp"
#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_node");
    
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");

    D2cControl d2c(&nh, &nh_priv);
    ros::spin();
    return 0;
}
