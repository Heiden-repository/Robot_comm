#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <thread>
#include <mutex>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <tf/transform_broadcaster.h>

#define max_t_vel 1500.0
#define max_r_vel 50.0
#define wheelsize 204.0
#define wheelbase 550.0
#define gear_ratio 30.0
#define stop_no_cmd 3.0

#define covar_const_right 1
#define covar_const_left 1

class CA_bot
{
private:
    struct Motor_state
    {
        short L_rpm;
        short L_current;
        char L_state;
        int L_pose;

        short R_rpm;
        short R_current;
        char R_state;
        int R_pose;
    };

    struct IByte
    {
        unsigned char byLow;
        unsigned char byHigh;
    };

    ros::NodeHandle nh_;
    int serial_port;
    std::string topic_name;

    int duration_publisher;
    bool robot_initialized;

    double RPM2VEL;
    double PPR;
    double PULSE2DIST;
    double dist_R, dist_L;

    unsigned char send_serial_protocol[13] = {0,};
    unsigned char motor_state_id[5] = {184, 183, 1, 210, 18};
    Motor_state prev_motor_state;

    double _x, _y, _th;
    cv::Mat _covar = cv::Mat::zeros(3, 3, CV_64F);

    double counter2dist;
    double angle2radian;

    boost::array<double, 36UL> _covariance;
    std::chrono::system_clock::time_point read_time, last_time, send_time;
    ros::Time current_time, prev_time;
    tf::TransformBroadcaster odom_broadcaster;

    //Publisher
    ros::Publisher odom_pub_;

    //Subscriber
    ros::Subscriber twist_msg_sub_;

    float twist_linear;
    float twist_angular;

    void initValue(void);
    void initSubscriber(ros::NodeHandle &nh_);
    void initPublisher(ros::NodeHandle &nh_);

    bool serial_connect(void);
    void send_serial(void);
    bool recieve_motor_state(void);;
    void odom_generator(double& difference_Lencoder,double& difference_Rencoder);

    void angleRearange();

    void odom_arrange(tf::TransformBroadcaster& odom_broadcaster);
    void make_covariance(double& gap_x, double& gap_y, double& gap_dist, double& for_covarian_radian);
    void make_velocity(double linear_vel, double angular_vel);


    short Byte2Short(unsigned char low, unsigned char high);
    int Byte2Int(unsigned char data1, unsigned char data2,unsigned char data3, unsigned char data4);
    IByte Int2Byte(short nln);
    unsigned char CalcChecksum(unsigned char* data, int leng);

    void twist_msg_callback(const geometry_msgs::Twist::ConstPtr &_twist_msg);
    void twist_convert_cmd_vel(float& Linear_serial, float& angular_serial);
    
public:
    int LeftEncoder, RightEncoder;
 
    void runLoop(void);

    CA_bot(ros::NodeHandle &_nh):
    nh_(_nh),dist_R(0.0),dist_L(0.0),_x(0.0),_y(0.0),_th(0.0),serial_port(0),twist_linear(0.0),twist_angular(0.0)
    {    
        initValue();
        initSubscriber(nh_);
        initPublisher(nh_);
        serial_connect();
    }

    ~CA_bot()
    {
        close(serial_port);
    }
};
