#include <ros/ros.h>
#include "robot/chic_m4k.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chic_m4k");

    ros::NodeHandle n;

    ROS_INFO("%s", ros::this_node::getName().c_str());

    Chic_m4k robot_cmm(n);
    robot_cmm.runLoop();
    
    return 0;
}