#include "robot/chic_m4k.hpp"

void Chic_m4k::initValue()
{
    
}

void Chic_m4k::initSubscriber(ros::NodeHandle &nh_)
{
    twist_msg_sub_ = nh_.subscribe("/cmd_vel",10,&Chic_m4k::twist_msg_callback,this);
}

void Chic_m4k::initPublisher(ros::NodeHandle &nh_)
{
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
}

void Chic_m4k::twist_msg_callback(const geometry_msgs::Twist::ConstPtr &_twist_msg)
{
    twist_linear = _twist_msg->linear.x;
    twist_angular = _twist_msg->angular.z;
    twist_convert_cmd_vel(twist_linear,twist_angular);
}

void Chic_m4k::twist_convert_cmd_vel(float &twist_linear, float &twist_angular)
{
    Linear_serial = (twist_linear * 60.0f / CV_PI / wheelsize * 2.0f / 3.0f ) * 1.3f + 127.0f;
    angular_serial = -(twist_angular * radpersec_to_RPM * 10 *2.0f / 3.0f) + 127.0f;
}

bool Chic_m4k::serial_connect()
{
    serial_port = 0;

    while (ros::ok())
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

    printf("Robot connection\n");
}

void Chic_m4k::send_receive_serial()
{
        encoder_mtx.lock();
        memset(send_serial_protocol, 0, send_serial_protocol_size);
        send_serial_protocol[0] = 0xFF;
        send_serial_protocol[1] = 0x07;
        send_serial_protocol[2] = 0x04;
        send_serial_protocol[3] = 0x05;

        send_serial_protocol[4] = (unsigned char)Linear_serial;
        send_serial_protocol[5] = (unsigned char)angular_serial;
        send_serial_protocol[6] = CalcChecksum(send_serial_protocol, send_serial_protocol_size);

        if (serial_port > 0)
        {
            int write_size = write(serial_port, send_serial_protocol, send_serial_protocol_size);

//            printf("write_size : %d\n", write_size);
        }

        memset(receive_serial_protocol, 0, recv_serial_protocol_size);

        int read_size = read(serial_port, receive_serial_protocol, recv_serial_protocol_size);

        //printf("read_size : %d\n", read_size);
        // for (int i = 0; i < read_size; i++)
        //     printf("receive_serial_protocol[%d] : %u\n", i, receive_serial_protocol[i]);

        encoder_mtx.unlock();
}

void Chic_m4k::get_val()
{
}

void Chic_m4k::receive_encoder()
{

    int receive_data = -1;
    int buf_end = 0;
    unsigned char buf[256];
    unsigned char temp_buf[256];
    receive_data = read(serial_port, temp_buf, 256);
    
    memcpy(&buf[buf_end], temp_buf, receive_data);
    buf_end += receive_data;

    for (int i = 0; i < buf_end; i++)
    {
        if (i + 13 > buf_end)
            break;

        if (buf[i] == 0xFF)
            if (buf[i + 1] == 0x0D)
                if (buf[i + 2] == 0x03)
                    if (buf[i + 3] == 0x05)
                    {
                        int s1 = buf[i + 4] & 0xFF;
                        int s2 = buf[i + 5] & 0xFF;
                        int s3 = buf[i + 6] & 0xFF;
                        int s4 = buf[i + 7] & 0xFF;
                        LEncoder = ((s1 << 24) + (s2 << 16) + (s3 << 8) + (s4 << 0));

                        s1 = buf[i + 8] & 0xFF;
                        s2 = buf[i + 9] & 0xFF;
                        s3 = buf[i + 10] & 0xFF;
                        s4 = buf[i + 11] & 0xFF;
                        REncoder = ((s1 << 24) + (s2 << 16) + (s3 << 8) + (s4 << 0));

                        int remain_size = buf_end - i - 13;
                        //printf("LEncoder : %5d REncoder : %5d\n", LEncoder, REncoder);
                        count_revolution();
                        if (remain_size != 0)
                        {
                            memcpy(temp_buf, buf, 256);
                            memcpy(buf, &temp_buf[i + 13], remain_size);
                            i = -1;
                            buf_end = remain_size;
                        }
                        else
                        {
                            buf_end = 0;
                            break;
                        }
                    }
    }
}

void Chic_m4k::count_revolution()
{
    if (prev_LEncoder == max_encoder_output+1)
        prev_LEncoder = LEncoder;

    int dL_Enc = LEncoder - prev_LEncoder;
    if (abs(dL_Enc) > max_encoder_value_change)
    {
        if (dL_Enc > 0)
            dL_Enc -= max_encoder_output;
        else
            dL_Enc += max_encoder_output;
    }
    //Lencoder_change += (dL_Enc / 10);


    if (prev_REncoder == max_encoder_output+1)
        prev_REncoder = REncoder;
    
    int dR_Enc = REncoder - prev_REncoder;
    if (abs(dR_Enc) > max_encoder_value_change)
    {
        if (dR_Enc > 0)
            dR_Enc -= max_encoder_output;
        else
            dR_Enc += max_encoder_output;
    }
    dR_Enc = -dR_Enc;

    //Rencoder_change += (dR_Enc / 10);

    //printf("Lencoder_change: %d    Rencoder_change: %d   \n", Lencoder_change, Rencoder_change);
    //printf("%5d %5d %5d %5d\n", dL_Enc, dR_Enc, LEncoder, REncoder);
    odom_generator(dL_Enc, dR_Enc);

    prev_LEncoder = LEncoder;
    prev_REncoder = REncoder;

    return;

    // int difference_Lencoder = Lencoder_change - temp_Lencoder_change;
    // int difference_Rencoder = Rencoder_change - temp_Rencoder_change;

    // temp_Lencoder_change = Lencoder_change;
    // temp_Rencoder_change = Rencoder_change;

    // odom_generator(difference_Lencoder, difference_Rencoder);
}

// void Chic_m4k::odom_generator(int& difference_Lencoder, int& difference_Rencoder)
// {
//     counter2dist = (wheelsize * CV_PI) / (double)encoder_per_wheel;

//     double difference_Lencoder_double = difference_Lencoder;
//     double difference_Rencoder_double = difference_Rencoder;

//     dist_L = difference_Lencoder_double * counter2dist;
//     dist_R = difference_Rencoder_double * counter2dist;

//     // printf("ddifference_Lencoder_Enc : %d difference_Rencoder : %d\n", difference_Lencoder, difference_Rencoder);
//     // printf("dist_R : %10lf dist_L : %10lf\n",dist_L, dist_R);

//     double gap_radian = (dist_R - dist_L) / wheelbase;
//     double gap_dist = (dist_R + dist_L) / 2.0;
//     double gap_x = cos(gap_radian) * gap_dist;
//     double gap_y = sin(gap_radian) * gap_dist;

//     // printf("%5d %5d. %10.6lf %10.6lf %10.6lf\n", 
//     //     difference_Lencoder, difference_Rencoder,
//     //     gap_x, gap_y, gap_radian);

//     add_motion(gap_x, gap_y, gap_radian);
// }

void Chic_m4k::odom_generator(int& difference_Lencoder, int& difference_Rencoder)
{
    counter2dist = (wheelsize * CV_PI) / (double)encoder_per_wheel;

    double difference_Lencoder_double = difference_Lencoder;
    double difference_Rencoder_double = difference_Rencoder;

    dist_L = difference_Lencoder_double * counter2dist;
    dist_R = difference_Rencoder_double * counter2dist;

    //printf("difference_Lencoder_Enc : %d difference_Rencoder : %d ", difference_Lencoder, difference_Rencoder);
    //printf("dist_R : %3.2lf dist_L : %3.2lf\n",dist_R, dist_L);

    double gap_radian = -(dist_R - dist_L) / wheelbase;
    double gap_dist = (dist_R + dist_L) / 2.0;
    double for_covarian_radian = gap_radian / 2.0 + _th;
    _th = gap_radian + _th;
    angleRearange();

    double gap_x = cos(for_covarian_radian) * gap_dist;
    double gap_y = sin(for_covarian_radian) * gap_dist;

    make_covariance(gap_x, gap_y, gap_dist, for_covarian_radian);
    _x += gap_x;
    _y += gap_y;

    //double degree_th = _th / PI * 180.0;
    printf("_x : %3.2lf _y : %3.2lf _th : %3.2lf \n", _x, _y, _th);
}

// void Chic_m4k::add_motion(double &x, double &y, double &th)
// {
//     _th = _th + th;

//     angleRearange();
//  //   double base_radian_th = angle2radian * _th;

//     _x = _x + x * cos(_th) - y * sin(_th);
//     _y = _y + x * sin(_th) + y * cos(_th);

//     printf("_x : %3.2lf _y : %3.2lf _th : %3.2lf \n", _x, _y, _th/CV_PI*180.0);
// }

void Chic_m4k::make_covariance(double& gap_x, double& gap_y,double& gap_dist, double& for_covarian_radian)
{
    cv::Mat error_pos = cv::Mat::eye(3, 3, CV_64F);
    error_pos.at<double>(0, 2) = gap_y;
    error_pos.at<double>(1, 2) = gap_x;

    cv::Mat error_motion(3, 2, CV_64F);
    error_motion.at<double>(0, 0) = cos(for_covarian_radian)/2 - gap_dist/wheelbase/2.0*sin(for_covarian_radian);
    error_motion.at<double>(0, 1) =  cos(for_covarian_radian)/2 + gap_dist/wheelbase/2.0*sin(for_covarian_radian);
    error_motion.at<double>(1, 0) =  sin(for_covarian_radian)/2 + gap_dist/wheelbase/2.0*cos(for_covarian_radian);
    error_motion.at<double>(1, 1) =  sin(for_covarian_radian)/2 - gap_dist/wheelbase/2.0*cos(for_covarian_radian);
    error_motion.at<double>(2, 0) = 1.0/wheelbase;
    error_motion.at<double>(2, 1) = -1.0/wheelbase;

    cv::Mat covar_for_count = cv::Mat::zeros(2,2,CV_64F);
    covar_for_count.at<double>(0,0) = covar_const_right * fabs(dist_R);
    covar_for_count.at<double>(1,1) = covar_const_left * fabs(dist_L);

    _covar = error_pos * _covar * error_pos + error_motion * covar_for_count * error_motion.t();

    _covariance[0] = _covar.at<double>(0,0);
    _covariance[1] = _covar.at<double>(0,1);
    _covariance[5] = _covar.at<double>(0,2);
    _covariance[6] = _covar.at<double>(1,0);
    _covariance[7] = _covar.at<double>(1,1);
    _covariance[11] = _covar.at<double>(1,2);
    _covariance[30] = _covar.at<double>(2,0);
    _covariance[31] = _covar.at<double>(2,1);
    _covariance[35] = _covar.at<double>(2,2);
//    printf("covariance");
}

void Chic_m4k::angleRearange()
{
    while (1)
    {
        if (_th > CV_PI)
        {
            _th -= 2.0*CV_PI;
            continue;
        }
        if (_th < -CV_PI)
        {
            _th += 2.0*CV_PI;
            continue;
        }
        break;
    }
}

unsigned char Chic_m4k::CalcChecksum(unsigned char *data, int leng)
{
    unsigned char csum;

    csum = 0xFF;
    for (; leng > 0; leng--)
        csum += *data++;

    return ~csum;
}

void Chic_m4k::odom_arrange(tf::TransformBroadcaster& odom_broadcaster)
{
    current_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = _x;
    odom_trans.transform.translation.y = _y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(_th), tf::Vector3(_x,_y, 0)), 
    // odom.pose.pose);

    //set the position
    odom.pose.pose.position.x = _x;
    odom.pose.pose.position.y = _y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = _covariance;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = twist_linear;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = twist_angular;
    odom.twist.covariance = _covariance;
    last_time = current_time;
    odom_pub_.publish(odom);
}

void Chic_m4k::runLoop()
{
    ros::Rate r(140);

    while (ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        send_receive_serial();
        receive_encoder();

        duration_publisher++;
        if (duration_publisher == 14)
        {
            odom_arrange(odom_broadcaster);

            duration_publisher = 0;
        }

        tcflush(serial_port, TCIFLUSH);

        r.sleep();
    };
}
