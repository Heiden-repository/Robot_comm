#include "robot/chic_m4k.hpp"

void Chic_m4k::initValue()
{

}

void Chic_m4k::initSubscriber()
{
    joy_msg_sub_ = nh_.subscribe("/joy",10, &Chic_m4k::joy_msg_callback, this);
}

void Chic_m4k::initPublisher()
{
    
}

void Chic_m4k::joy_msg_callback(const sensor_msgs::Joy::ConstPtr &_joy_msg)
{
    
}

void Chic_m4k::runLoop()
{
    ros::Rate r(1);

    while (ros::ok())
    {
        r.sleep();
        
        ros::spinOnce();
    };
}


