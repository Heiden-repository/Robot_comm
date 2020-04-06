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

#define JOY_BUTTON_AMOUNT 12
#define JOY_AXES_AMOUNT 6

#define send_serial_protocol_size 7
#define recv_serial_protocol_size 6
#define encoder_protocol_size 13
#define buffer_size 24
#define gear_ratio 6.2
#define velocity_zero 127
#define max_encoder_output 4096
#define max_encoder_value_change 2048

#define max_vel_x 0.5
#define min_vel_x 0.1
#define wheelsize 0.19
#define wheelbase 0.51

#define covar_const_right 1
#define covar_const_left 1

#define radpersec_to_RPM 9.54929659

class Chic_m4k
{
private:
    ros::NodeHandle nh_;
    int serial_port;
    std::string topic_name;

    float Linear_serial;
    float angular_serial;

    float twist_linear;
    float twist_angular;

    int LEncoder, REncoder;
    int prev_LEncoder, prev_REncoder;

    int duration_publisher;

    int encoder_per_wheel;
    int Lencoder_change, Rencoder_change;
    int temp_Lencoder_change, temp_Rencoder_change;
    double dist_R, dist_L;

    double _x, _y, _th;
    cv::Mat _covar = cv::Mat::zeros(3,3,CV_64F);

    double counter2dist;
    double angle2radian = CV_PI / 180.0;

    //double _covariance[36];
    boost::array<double,36UL> _covariance;
    ros::Time current_time, last_time;
    tf::TransformBroadcaster odom_broadcaster;

    //buffer
    unsigned char dataBuffer[buffer_size];

    //protocol
    unsigned char send_serial_protocol[send_serial_protocol_size];
    unsigned char receive_serial_protocol[recv_serial_protocol_size];
    unsigned char encoder_protocol[encoder_protocol_size];

    //Publisher
    ros::Publisher odom_pub_;

    //Subscriber
    ros::Subscriber twist_msg_sub_;

    std::mutex encoder_mtx;

    void initValue(void);
    void initSubscriber(ros::NodeHandle& nh_);
    void initPublisher(ros::NodeHandle &nh_);

    bool serial_connect(void);
    void send_receive_serial(void);
    void receive_encoder(void);
    void count_revolution(void);
    void odom_generator(int& difference_Lencoder,int& difference_Rencoder);
    void add_motion(double &x, double &y, double &th);

    void get_val();
    void angleRearange();

    void odom_arrange(tf::TransformBroadcaster& odom_broadcaster);
    void make_covariance(double& gap_x, double& gap_y, double& gap_dist, double& for_covarian_radian);

    unsigned char CalcChecksum(unsigned char* data, int leng);

    void twist_msg_callback(const geometry_msgs::Twist::ConstPtr &_twist_msg);
    void twist_convert_cmd_vel(float& Linear_serial, float& angular_serial);
    
public:
    int LeftEncoder, RightEncoder;
 
    void runLoop(void);

    Chic_m4k(ros::NodeHandle &_nh):
    nh_(_nh),Linear_serial(velocity_zero),angular_serial(velocity_zero),encoder_per_wheel(max_encoder_output*gear_ratio),_x(0.0),_y(0.0),_th(0.0),REncoder(0),LEncoder(0),_covariance({0,}),
    prev_LEncoder(max_encoder_output+1),prev_REncoder(max_encoder_output+1),LeftEncoder(0),RightEncoder(0),Lencoder_change(0),Rencoder_change(0),temp_Lencoder_change(0),temp_Rencoder_change(0),duration_publisher(0),current_time(ros::Time::now()),last_time(ros::Time::now())
    {
        initValue();
        initSubscriber(nh_);
        initPublisher(nh_);
        serial_connect();
    }

    ~Chic_m4k()
    {
        close(serial_port);
    }
};
