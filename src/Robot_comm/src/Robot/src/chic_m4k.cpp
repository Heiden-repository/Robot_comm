#include "robot/chic_m4k.hpp"

void Chic_m4k::initValue()
{
    
}

void Chic_m4k::initSubscriber(ros::NodeHandle &nh_)
{
    joy_msg_sub_ = nh_.subscribe("/joy", 10, &Chic_m4k::joy_msg_callback, this);
}

void Chic_m4k::initPublisher()
{

}

void Chic_m4k::joy_msg_callback(const sensor_msgs::Joy::ConstPtr &_joy_msg)
{
    joy_msg.seq = _joy_msg->header.seq;
    joy_msg.axes[0] = _joy_msg->axes[0];
    joy_msg.axes[1] = _joy_msg->axes[1];

    joy_msg.buttons[6] = _joy_msg->buttons[6];
}

bool Chic_m4k::serial_connect()
{
    serial_port = 0;
	struct termios tty;

	while(ros::ok())
	{
		serial_port = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );
		if(serial_port<0)
		{ 
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            printf("Error %i from open: %s\n", errno, strerror(errno));
		}
        else
            break;
    }
    ROS_INFO("Robot_connection");
    ROS_INFO("serial_port in serial_connect : %d", serial_port);
}

void Chic_m4k::send_serial()
{
    serial_protocol[0] = 0xFF;
    serial_protocol[1] = 0x07;
    serial_protocol[2] = 0x04;
    serial_protocol[3] = 0x05;

    serial_protocol[4] = Linear_velocity;
    serial_protocol[5] = angular_velocity;

    serial_protocol[6] = CalcChecksum(serial_protocol, serial_protocol_size);

    ROS_INFO("serial_port in send_serial : %d", serial_port);

    if (serial_port > 0)
    {
        int val = write(serial_port, serial_protocol, serial_protocol_size);
        ROS_INFO("send data size : %d", val);
    }
}

void Chic_m4k::receive_serial()
{
    
}

void Chic_m4k::send_receive_serial()
{
    
}

unsigned char Chic_m4k::CalcChecksum(unsigned char* data, int leng)
{
    unsigned char csum;

    csum = 0xFF;
    for(;leng>0; leng--)
        csum += *data++;

    return ~csum;
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


