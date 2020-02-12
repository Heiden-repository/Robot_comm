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

    struct termios termi;

    memset( &termi, 0, sizeof(termi) );

    termi.c_cflag = B115200; //1 stop bit
    termi.c_cflag |= CS8;    // data length 8bit
    termi.c_cflag |= CLOCAL; // Use iternel commutication port
    termi.c_cflag |= CREAD;  // enable read & write
    termi.c_iflag = 0;       // no parity bit
    termi.c_oflag = 0;
    termi.c_lflag = 0;
    termi.c_cc[VTIME] = 0;
    termi.c_cc[VMIN] = 0;

    tcflush(serial_port, TCIFLUSH);
    tcsetattr(serial_port, TCSANOW, &termi);

    std::cout << "Robot connection" << std::endl;
}

void Chic_m4k::send_serial()
{
    memset(send_serial_protocol, 0, serial_protocol_size);
    send_serial_protocol[0] = 0xFF;
    send_serial_protocol[1] = 0x07;
    send_serial_protocol[2] = 0x04;
    send_serial_protocol[3] = 0x05;

    send_serial_protocol[4] = Linear_velocity;
    send_serial_protocol[5] = angular_velocity;

    send_serial_protocol[6] = CalcChecksum(send_serial_protocol, serial_protocol_size);

    if (serial_port > 0)
    {
        int val = write(serial_port, send_serial_protocol, serial_protocol_size);
    }
}

void Chic_m4k::receive_serial()
{
    memset(receive_serial_protocol, 0, serial_protocol_size);

    int read_size = read(serial_port, receive_serial_protocol, 24);

    for (int i = 0; i < read_size; i++)
        std::cout << "receive_serial_protocol[" << i << "] : " << receive_serial_protocol[i] << std::endl;
}

void Chic_m4k::send_receive_serial()
{

    send_thread = std::thread([&]() {
        ros::Rate rate(1);
        while (ros::ok())
        {
            send_serial_protocol[0] = 0xFF << 4;
            send_serial_protocol[0] = 0x07;
            send_serial_protocol[1] = 0x04 << 4;
            send_serial_protocol[1] = 0x05;
            send_serial_protocol[2] = Linear_velocity;
            send_serial_protocol[3] = angular_velocity;
            send_serial_protocol[4] = CalcChecksum(send_serial_protocol, serial_protocol_size);

            if(0)
            {
            send_serial_protocol[0] = 0xFF;
            send_serial_protocol[1] = 0x07;
            send_serial_protocol[2] = 0x04;
            send_serial_protocol[3] = 0x05;

            send_serial_protocol[4] = Linear_velocity;
            send_serial_protocol[5] = angular_velocity;

            send_serial_protocol[6] = CalcChecksum(send_serial_protocol, serial_protocol_size);
            }
            
            if (serial_port > 0)
            {
                //int val = write(serial_port, send_serial_protocol, serial_protocol_size);
                int val = write(serial_port, send_serial_protocol, 5);

                std::cout << "val : " << val << std::endl;
            }
            rate.sleep();
        }
    });

    recv_thread = std::thread([&]() {
        while (ros::ok())
        {
            memset(receive_serial_protocol, 0, serial_protocol_size);

            int read_size = read(serial_port, receive_serial_protocol, 24);
            if(read_size > 0)
                std::cout << "serial_read_size : " << read_size << std::endl;
        }
    });
}

void Chic_m4k::receive_encoder()
{
    recv_encoder_thread = std::thread([&]() {
        while (ros::ok())
        {
            memset(encoder_protocol, 0, serial_protocol_size);

            int read_size = read(serial_port, encoder_protocol, 24);
            if (read_size > 0)
            {
                std::cout << "encoder_read_size : " << read_size << std::endl;
                for (int i = 0; i < read_size; i++)
                    std::cout << "encoder_protocol[" << i << "] : " << encoder_protocol[i] << std::endl;
            }
        }
    });
}

unsigned char Chic_m4k::CalcChecksum(unsigned char *data, int leng)
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


