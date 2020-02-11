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

    // memset(&tty, 0, sizeof(tty));

    // if (tcgetattr(serial_port, &tty) != 0)
    // {
    //     printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    // }

    // tty.c_cflag &= ~PARENB;
    // tty.c_cflag &= ~CSTOPB;
    // tty.c_cflag |= CS7;
    // tty.c_cflag &= ~CRTSCTS;

    // // tty.c_cflag = B57600;  //1 stop bit
    // tty.c_cflag |= CS7;    // data length 8bit
    // tty.c_cflag |= CLOCAL; // Use iternel commutication port
    // tty.c_cflag |= CREAD;  // enable read & write
    // // tty.c_iflag = 0;       // no parity bit
    // // tty.c_oflag = 0;
    // // tty.c_lflag = 0;
    // // tty.c_cc[VTIME] = 0;
    // // tty.c_cc[VMIN] = 0;

    // tcflush (serial_port, TCIFLUSH );
    // tcsetattr(serial_port, TCSANOW, &tty );

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


