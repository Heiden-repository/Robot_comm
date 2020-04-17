#include "robot/CA_bot.hpp"
#include <time.h>
void CA_bot::initValue()
{
    RPM2VEL = (wheelsize * CV_PI) / (60.0 * gear_ratio);
    PPR = 24 * gear_ratio;
    PULSE2DIST = (wheelsize * CV_PI) / PPR;
    angle2radian = CV_PI / 180.0;
    robot_initialized = false;

    read_time = std::chrono::system_clock::now();
    last_time = std::chrono::system_clock::now();
    send_time = std::chrono::system_clock::now();
}

void CA_bot::initSubscriber(ros::NodeHandle &nh_)
{
    twist_msg_sub_ = nh_.subscribe("/cmd_vel",10,&CA_bot::twist_msg_callback,this);
}

void CA_bot::initPublisher(ros::NodeHandle &nh_)
{
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
}

void CA_bot::twist_msg_callback(const geometry_msgs::Twist::ConstPtr &_twist_msg)
{
    twist_linear = _twist_msg->linear.x * 1000.0;
    twist_angular = _twist_msg->angular.z;
    send_serial();
}

void CA_bot::twist_convert_cmd_vel(float &twist_linear, float &twist_angular)
{

}

bool CA_bot::serial_connect()
{
	struct termios newtio;

	while(ros::ok())
	{
		serial_port = open( "/dev/ttyS0", O_RDWR | O_NOCTTY );
		if(serial_port<0) 
		{ 
			ROS_ERROR("[CAbot] connecion error. retry");
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		else break;
	}

	ROS_INFO("[CAbot] connetion established.");
    memset( &newtio, 0, sizeof(newtio) );

	newtio.c_cflag = B57600;	//1 stop bit
  	newtio.c_cflag |= CS8;      // data length 8bit 
    newtio.c_cflag |= CLOCAL;	// Use iternel commutication port
    newtio.c_cflag |= CREAD;	// enable read & write
    newtio.c_iflag = 0;			// no parity bit
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush (serial_port, TCIFLUSH );
    tcsetattr(serial_port, TCSANOW, &newtio );
	//init Serial field

    memset(send_serial_protocol, 0, 13);

	send_serial_protocol[0] = 183;
	send_serial_protocol[1] = 184;
	send_serial_protocol[2] = 254;
	send_serial_protocol[3] = 207;
	send_serial_protocol[4] = 7;
	send_serial_protocol[5] = 1;
	send_serial_protocol[6] = 0;		
	send_serial_protocol[7] = 0;		
	send_serial_protocol[8] = 2; 
	send_serial_protocol[9] = 0;	
	send_serial_protocol[10] = 0;	
	send_serial_protocol[11] = 0; 
	send_serial_protocol[12] = 186; 
}

void CA_bot::send_serial()
{
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
		std::chrono::duration<double> dt = now - send_time;
		if(dt.count() > stop_no_cmd)

		if(serial_port > 0) int val = write(serial_port, send_serial_protocol, 13);
        
}

void CA_bot::make_velocity(double linear_vel, double angular_vel)
{
    if(fabs(linear_vel) > max_t_vel)
	{
		ROS_WARN("[CAbot] t_vel: %lf > max_t_vel: %lf", linear_vel, max_t_vel);
		if(linear_vel > 0) linear_vel = max_t_vel;
		else linear_vel = -max_t_vel;
	}
	if(fabs(angular_vel) > max_r_vel)
	{
		ROS_WARN("[CAbot] r_vel: %lf > max_r_vel: %lf", angular_vel, max_r_vel);
		if(angular_vel > 0) angular_vel = max_r_vel;
		else angular_vel = -max_r_vel;
	}

	ROS_INFO("linear_vel:%lf angular_vel:%lf", linear_vel, angular_vel);

	double VEL_DIFF = wheelbase*angular_vel*0.5;

	double LRPM = (linear_vel - VEL_DIFF)/RPM2VEL;
	double RRPM = (linear_vel + VEL_DIFF)/RPM2VEL;

	short Lnl = (short)LRPM;
	short Rnl = (short)RRPM;
//////////////////////////////////////
	IByte Lnlbyte, Rnlbyte;
	Lnlbyte = Int2Byte(-Lnl);
	Rnlbyte = Int2Byte(Rnl);

	bool stop_sig = false;
	char vel_arr[5] = {};

	if(LRPM == 0.0 && RRPM == 0.0)	stop_sig = true;
	if(stop_sig)
	{
		for(int i = 0; i < 4; i++)
			vel_arr[i] = 0;
	}
	else
	{
		vel_arr[0] = Lnlbyte.byLow;		//L motor
		vel_arr[1] = Lnlbyte.byHigh;	
		vel_arr[2] = Rnlbyte.byLow;		//R motor	
		vel_arr[3] = Rnlbyte.byHigh;	
	}
	
	send_serial_protocol[0] = 183;
	send_serial_protocol[6] = vel_arr[0];
	send_serial_protocol[7] = vel_arr[1];
	send_serial_protocol[9] = vel_arr[2];
	send_serial_protocol[10] = vel_arr[3];

	unsigned char chk = 0;
	for(int i = 0; i<12;i++)
	{
		chk += *(send_serial_protocol +i);		
	}
	chk = (~chk + 1);
	send_serial_protocol[12] = chk;


	send_time = std::chrono::system_clock::now();
}

bool CA_bot::recieve_motor_state(void)
{

    if(serial_port <= 0) return 0;

	unsigned char read_motor_cmd[8] = {};
	read_motor_cmd[0] = 183;
	read_motor_cmd[1] = 184;
	read_motor_cmd[2] = 1;
	read_motor_cmd[3] = 4;
	read_motor_cmd[4] = 1;
	read_motor_cmd[5] = 210;
	read_motor_cmd[6] = 185;

	unsigned char chk = 0;
	for(int i = 0; i<7;i++)
		chk += *(read_motor_cmd +i);		
	chk = (~chk + 1);
	read_motor_cmd[7] = chk;

	if(serial_port > 0) int val = write(serial_port, read_motor_cmd, 8);
	
	unsigned char received_serial[24];
	memset(received_serial, 0, 24);

	std::chrono::duration<double> dt_duration = read_time - last_time;
	last_time = read_time;

	double dt = dt_duration.count();

	int read_size = read(serial_port, received_serial, 24);

	bool check_id = true;
	for(int i=0; i<5; i++)
		if(motor_state_id[i] != received_serial[i]) check_id = false;

	if(read_size < 5 || !check_id)
	{
		if(robot_initialized) ROS_WARN("[CAbot] read moter state fail");
		return 0;
	}

    Motor_state curr_motor_state;
	curr_motor_state.L_rpm = -Byte2Short(received_serial[5], received_serial[6]);
	curr_motor_state.L_current = -Byte2Short(received_serial[7], received_serial[8]);
	curr_motor_state.L_state = received_serial[9];
	curr_motor_state.L_pose = -Byte2Int(received_serial[10], received_serial[11], received_serial[12], received_serial[13]);

	curr_motor_state.R_rpm = Byte2Short(received_serial[14], received_serial[15]);
	curr_motor_state.R_current = Byte2Short(received_serial[16], received_serial[17]);
	curr_motor_state.R_state = received_serial[18];
	curr_motor_state.R_pose = Byte2Int(received_serial[19], received_serial[20], received_serial[21], received_serial[22]);

	int L_d_pose = curr_motor_state.L_pose - prev_motor_state.L_pose;
	int R_d_pose = curr_motor_state.R_pose - prev_motor_state.R_pose;

	int L_d_rpm = (int)(curr_motor_state.L_rpm - prev_motor_state.L_rpm);
	int R_d_rpm = (int)(curr_motor_state.R_rpm - prev_motor_state.R_rpm);

	memcpy(&prev_motor_state, &curr_motor_state, sizeof(Motor_state));	

	if(!robot_initialized) 
	{
		robot_initialized = true;
		return 0;
	}

	double L_d_distance = (double)L_d_pose*PULSE2DIST;
	double R_d_distance = (double)R_d_pose*PULSE2DIST;

    odom_generator(L_d_distance,R_d_distance);

	return 1;
}


void CA_bot::odom_generator(double& dist_L, double& dist_R)
{
    //printf("dist_R : %3.2lf dist_L : %3.2lf\n",dist_R, dist_L);

    double gap_radian = (dist_R - dist_L) / wheelbase;
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


void CA_bot::make_covariance(double& gap_x, double& gap_y,double& gap_dist, double& for_covarian_radian)
{
    double covari_dist_L = (int)(dist_L*1000)/1000.0f;
    double covari_dist_R = (int)(dist_R*1000)/1000.0f;


    cv::Mat error_pos = cv::Mat::eye(3, 3, CV_64F);
    error_pos.at<double>(0, 2) = -gap_y;
    error_pos.at<double>(1, 2) = gap_x;

    cv::Mat error_motion(3, 2, CV_64F);
    error_motion.at<double>(0, 0) = cos(for_covarian_radian)/2 - gap_dist/wheelbase/2*sin(for_covarian_radian);
    error_motion.at<double>(0, 1) =  cos(for_covarian_radian)/2 + gap_dist/wheelbase/2*sin(for_covarian_radian);
    error_motion.at<double>(1, 0) =  sin(for_covarian_radian)/2 + gap_dist/wheelbase/2*cos(for_covarian_radian);
    error_motion.at<double>(1, 1) =  sin(for_covarian_radian)/2 - gap_dist/wheelbase/2*cos(for_covarian_radian);
    error_motion.at<double>(2, 0) = 1/wheelbase;
    error_motion.at<double>(2, 1) = -1/wheelbase;
    cv::Mat covar_for_count = cv::Mat::zeros(2,2,CV_64F);
    covar_for_count.at<double>(0,0) = covar_const_right * fabs(covari_dist_R);
    covar_for_count.at<double>(1,1) = covar_const_left * fabs(covari_dist_L);

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
}

void CA_bot::angleRearange()
{
    while (1)
    {
        if (_th > CV_PI)
        {
            _th -= 2*CV_PI;
            continue;
        }
        if (_th < -CV_PI)
        {
            _th += 2*CV_PI;
            continue;
        }
        break;
    }
}

CA_bot::IByte CA_bot::Int2Byte(short nln)
{
	IByte Ret;

	Ret.byLow = nln & 0xff;
	Ret.byHigh = nln >>8 & 0xff;
	return Ret;
}

short CA_bot::Byte2Short(unsigned char low, unsigned char high)
{
	return ((unsigned short)low | (unsigned short)high<<8);
}

int CA_bot::Byte2Int(unsigned char data1, unsigned char data2, 
		unsigned char data3, unsigned char data4)
{
	return ((unsigned int)data1 | (unsigned int)data2<<8 | 
			(unsigned int)data3<<16 | (unsigned int)data4<<24);
}

unsigned char CA_bot::CalcChecksum(unsigned char *data, int leng)
{
    unsigned char csum;

    csum = 0xFF;
    for (; leng > 0; leng--)
        csum += *data++; 

    return ~csum;
}

void CA_bot::odom_arrange(tf::TransformBroadcaster& odom_broadcaster)
{
    current_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = _x / 1000.0;
    odom_trans.transform.translation.y = _y / 1000.0;
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
    odom.pose.pose.position.x = _x / 1000.0;
    odom.pose.pose.position.y = _y / 1000.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = _covariance;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = twist_linear;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = twist_angular;
    odom.twist.covariance = _covariance;
    prev_time = current_time;
    odom_pub_.publish(odom);
}

void CA_bot::runLoop()
{
    ros::Rate r(140);

	while (ros::ok())
    {
       
        ros::spinOnce();
        current_time = ros::Time::now();
        recieve_motor_state();

        duration_publisher++;
        if (duration_publisher == 12)
        {
            odom_arrange(odom_broadcaster);
			
            duration_publisher = 0;
        }
        tcflush(serial_port, TCIFLUSH);
		//printf("ros::Time.back: %d\n",ros::Time::now().nsec);

        r.sleep();
    };
}
