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
    // encoder_thread = std::thread([&]() {
        encoder_mtx.lock();
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

            //printf("write_size : %d\n", write_size);
        }

        memset(receive_serial_protocol, 0, recv_serial_protocol_size);

        int read_size = read(serial_port, receive_serial_protocol, recv_serial_protocol_size);

        //printf("read_size : %d\n", read_size);
        //for (int i = 0; i < read_size; i++)
        //    printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);

        encoder_mtx.unlock();
    // });
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

    int read_size = read(serial_port, receive_serial_protocol, recv_serial_protocol_size);
    if (read_size > 0)
    {
        printf("read_size : %d\n", read_size);
        for (int i = 0; i < read_size; i++)
            printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);
    }
    while(true)
    {
        int encoder_read_size = read(serial_port, encoder_protocol, encoder_protocol_size);
        for (int i = 0; i < encoder_read_size; i++)
            printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);
    }
}

void Chic_m4k::get_val()
{

}

void Chic_m4k::receive_encoder()
{
    unsigned char start[1];
    unsigned char length[1];
    memset(encoder_protocol, 0, encoder_protocol_size);

    while (ros::ok())
    {
      int read_start = read(serial_port, start, 1);  
      if(read_start == 1)
      {
          if(start[0] == 0xFF)
          {
              //printf("get start 1byte  ");
              int read_data_length = read(serial_port, length, 1);
              if(read_data_length == 1)
              {
                  if(length[0] == 0x0D)
                  {
                      //printf("get datalength 1byte  :");
                      int data_size = read(serial_port, encoder_protocol, 11);
                      if(data_size == 11)
                      {
                            //for (int i = 0; i < data_size; i++)
                            //    printf("encoder_protocol[%d] : %u\n", i, encoder_protocol[i]);
                            int s1 = encoder_protocol[2] & 0xFF;
                            int s2 = encoder_protocol[3] & 0xFF;
                            int s3 = encoder_protocol[4] & 0xFF;
                            int s4 = encoder_protocol[5] & 0xFF;
                            LEncoder = ((s1 << 24) + (s2 << 16) + (s3 << 8) + (s4 << 0));

                            s1 = encoder_protocol[6] & 0xFF;
                            s2 = encoder_protocol[7] & 0xFF;
                            s3 = encoder_protocol[8] & 0xFF;
                            s4 = encoder_protocol[9] & 0xFF;
                            REncoder = ((s1 << 24) + (s2 << 16) + (s3 << 8) + (s4 << 0));

                            
                            //std::memcpy(&LEncoder, &encoder_protocol[2], sizeof(int));
                            //std::memcpy(&REncoder, &encoder_protocol[6], sizeof(int));
                            printf("Lencoder: %u    Rencoder: %u   \n", LEncoder, REncoder); 
                            break;
                      }
                  }
                  else{
                      break;
                  }
              }
          }

      }
    }

    // encoder_thread = std::thread([&]() {
    //     ros::Rate rate(10);
    //     while (ros::ok())
    //     {
    //         memset(encoder_protocol, 0, encoder_protocol_size);

    //         int encoder_read_size = read(serial_port, encoder_protocol, encoder_protocol_size);
    //         for (int i = 0; i < encoder_read_size; i++)
    //             printf("encoder_protocol[%d] : %u\n", i, encoder_protocol[i]);
    //         rate.sleep();
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
    ros::Rate r(30);

    while (ros::ok())
    {
            send_receive_serial();
    receive_encoder();
            
        tcflush(serial_port, TCIFLUSH);

        r.sleep();
        
        ros::spinOnce();
    };
}


