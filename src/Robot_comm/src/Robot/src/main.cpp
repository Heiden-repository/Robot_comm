#include <ros/ros.h>
#include "robot/CA_bot.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CA_bot");

    ros::NodeHandle n;

    ROS_INFO("%s", ros::this_node::getName().c_str());

    CA_bot robot_cmm(n);

    robot_cmm.runLoop();
    
    return 0;
}