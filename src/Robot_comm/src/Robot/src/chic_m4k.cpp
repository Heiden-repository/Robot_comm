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
    angular = _joy_msg->axes[0];
    linear = _joy_msg->axes[1];

    toggle_button = _joy_msg->buttons[6];
    convert_cmd_vel();
    send_receive_serial();
}

void Chic_m4k::convert_cmd_vel()
{
    if(toggle_button)
    {
        Linear_velocity = (linear + 1)*127;
        angular_velocity = (angular + 1)*127;
    }
    else
    {
        Linear_velocity = 127;
        angular_velocity = 127;
    }
}

bool Chic_m4k::serial_connect()
{
    serial_port = 0;

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

    termi.c_cflag = B115200; 
    termi.c_cflag |= CS8;    
    termi.c_cflag |= CLOCAL; 
    termi.c_cflag |= CREAD;  
    termi.c_iflag = 0;       
    termi.c_oflag = 0;
    termi.c_lflag = 0;
    termi.c_cc[VTIME] = 0;
    termi.c_cc[VMIN] = 0;

    tcflush(serial_port, TCIFLUSH);
    tcsetattr(serial_port, TCSANOW, &termi);

    printf("Robot connection");
}

void Chic_m4k::send_serial()
{
    memset(send_serial_protocol, 0, send_serial_protocol_size);
    send_serial_protocol[0] = 0xFF;
    send_serial_protocol[1] = 0x07;
    send_serial_protocol[2] = 0x04;
    send_serial_protocol[3] = 0x05;

    send_serial_protocol[4] = (unsigned char)Linear_velocity;
    send_serial_protocol[5] = (unsigned char)angular_velocity;
    send_serial_protocol[6] = CalcChecksum(send_serial_protocol, send_serial_protocol_size);

    if (serial_port > 0)
    {
        int write_size = write(serial_port, send_serial_protocol, send_serial_protocol_size);

        printf("write_size : %d\n", write_size);
    }
}

void Chic_m4k::receive_serial()
{
    memset(receive_serial_protocol, 0, recv_serial_protocol_size);

    int read_size = read(serial_port, receive_serial_protocol, 7);

    for (int i = 0; i < read_size; i++)
        printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);
}

void Chic_m4k::send_receive_serial()
{
    memset(send_serial_protocol, 0, send_serial_protocol_size);
    send_serial_protocol[0] = 0xFF;
    send_serial_protocol[1] = 0x07;
    send_serial_protocol[2] = 0x04;
    send_serial_protocol[3] = 0x05;

    send_serial_protocol[4] = (unsigned char)Linear_velocity;
    send_serial_protocol[5] = (unsigned char)angular_velocity;
    send_serial_protocol[6] = CalcChecksum(send_serial_protocol, send_serial_protocol_size);

    if (serial_port > 0)
    {
        int write_size = write(serial_port, send_serial_protocol, send_serial_protocol_size);

        printf("write_size : %d\n", write_size);
    }

    memset(receive_serial_protocol, 0, recv_serial_protocol_size);

    int read_size = read(serial_port, receive_serial_protocol, recv_serial_protocol_size);
    if (read_size > 0)
    {
        printf("read_size : %d\n", read_size);
        for (int i = 0; i < read_size; i++)
            printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);
    }
    encoder_thread = std::thread([&]() {
        ros::Rate rate(1);
        while (ros::ok())
        {
            memset(encoder_protocol, 0, encoder_protocol_size);

            int encoder_read_size = read(serial_port, encoder_protocol, encoder_protocol_size);
            for (int i = 0; i < encoder_read_size; i++)
                printf("encoder_protocol[%d] : %u\n", i, encoder_protocol[i]);
            rate.sleep();
        }
    });
}

void Chic_m4k::set_val(float Linear_velocity, float angular_velocity)
{
    memset(send_serial_protocol, 0, send_serial_protocol_size);
    send_serial_protocol[0] = 0xFF;
    send_serial_protocol[1] = 0x07;
    send_serial_protocol[2] = 0x04;
    send_serial_protocol[3] = 0x05;

    send_serial_protocol[4] = (unsigned char)Linear_velocity;
    send_serial_protocol[5] = (unsigned char)angular_velocity;
    send_serial_protocol[6] = CalcChecksum(send_serial_protocol, send_serial_protocol_size);

    if (serial_port > 0)
    {
        int write_size = write(serial_port, send_serial_protocol, send_serial_protocol_size);

        printf("write_size : %d\n", write_size);
    }

    memset(receive_serial_protocol, 0, recv_serial_protocol_size);

    int read_size = read(serial_port, receive_serial_protocol, 7);
    if (read_size > 0)
    {
        printf("read_size : %d\n", read_size);
        for (int i = 0; i < read_size; i++)
            printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);
    }
    while(true)
    {
        int encoder_read_size = read(serial_port, encoder_protocol, 13);
        for (int i = 0; i < encoder_read_size; i++)
            printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);
    }
}

void Chic_m4k::get_val()
{

}

void Chic_m4k::receive_encoder()
{
    // recv_encoder_thread = std::thread([&]() {
    //     while (ros::ok())
    //     {
    //         memset(encoder_protocol, 0, recv_serial_protocol_size);

    //         int read_size = read(serial_port, encoder_protocol, 24);
    //         if (read_size > 0)
    //         {
    //             std::cout << "encoder_read_size : " << read_size << std::endl;
    //             for (int i = 0; i < read_size; i++)
    //                      printf("encoder_protocol[%d] : %u\n",i,encoder_protocol[i]);
    //         }
    //     }
    // });
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


