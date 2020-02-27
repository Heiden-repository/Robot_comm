#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <thread>
#include <mutex>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "tf/transform_broadcaster.h"

#define JOY_BUTTON_AMOUNT 12
#define JOY_AXES_AMOUNT 6

#define send_serial_protocol_size 7
#define recv_serial_protocol_size 6
#define encoder_protocol_size 13
#define buffer_size 24
#define gear_ratio 6.2
#define velocity_zero 127
#define max_encoder_output 4096
#define max_encoder_value_change 2000
#define PI = 3.141592

class Chic_m4k
{
private:
    ros::NodeHandle nh_;
    int serial_port;
    std::string topic_name;

    float Linear_velocity;
    float angular_velocity;

    float linear;
    float angular;

    unsigned int LEncoder, REncoder;
    int prev_LEncoder, prev_REncoder;

    int duration_publisher;

    int encoder_per_wheel;
    int Lencoder_change,Rencoder_change;
    int temp_Lencoder_change,temp_Rencoder_change;
    double wheelsize,wheelbase;
    double dist_R,dist_L;
    
    double _x,_y,_th;
    
    bool toggle_button;

    double counter2dist;
    double angle2radian = 3.141592 / 180.0;

    //buffer
    unsigned char dataBuffer[buffer_size];

    //protocol
    unsigned char send_serial_protocol[send_serial_protocol_size];
    unsigned char receive_serial_protocol[recv_serial_protocol_size];
    unsigned char encoder_protocol[encoder_protocol_size];

    //Publisher
    ros::Publisher odom_pub_;

    //Subscriber
    ros::Subscriber joy_msg_sub_;
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
    void add_motion(double& gap_x,double& gap_y,double& gap_th);

    void set_val(float Linear_velocity,float angular_velocity);
    void get_val();
    void angleRearange();

    nav_msgs::Odometry odom_arrange();

    unsigned char CalcChecksum(unsigned char* data, int leng);

    void joy_msg_callback(const sensor_msgs::Joy::ConstPtr &_joy_msg);
    void twist_msg_callback(const geometry_msgs::Twist::ConstPtr &_twist_msg);
    void convert_cmd_vel();
    
public:
    int LeftEncoder, RightEncoder;

    void runLoop(void);

    Chic_m4k(ros::NodeHandle &_nh):
    nh_(_nh),toggle_button(0),linear(0),angular(0),Linear_velocity(velocity_zero),angular_velocity(velocity_zero),encoder_per_wheel(max_encoder_output*gear_ratio/10),
    prev_LEncoder(-1),prev_REncoder(-1),LeftEncoder(0),RightEncoder(0),Lencoder_change(0),Rencoder_change(0),wheelsize(0.19),wheelbase(0.51),temp_Lencoder_change(0),temp_Rencoder_change(0),duration_publisher(0)
    {
        initValue();
        initSubscriber(nh_);
        initPublisher(nh_);
        serial_connect();

        //set_val(255.0f,0.0f);
    }

    ~Chic_m4k()
    {
        close(serial_port);
    }
};