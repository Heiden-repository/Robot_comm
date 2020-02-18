#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <thread>
#include <mutex>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define JOY_BUTTON_AMOUNT 12
#define JOY_AXES_AMOUNT 6

#define send_serial_protocol_size 7
#define recv_serial_protocol_size 6
#define encoder_protocol_size 13
#define buffer_size 24
#define max_encoder_value_change 500
#define velocity_zero 127
#define max_encoder_output 4096

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
    int temp_LEncoder, temp_REncoder;

    int max_encoder;
    
    bool toggle_button;

    //buffer
    unsigned char dataBuffer[buffer_size];

    //protocol
    unsigned char send_serial_protocol[send_serial_protocol_size];
    unsigned char receive_serial_protocol[recv_serial_protocol_size];
    unsigned char encoder_protocol[encoder_protocol_size];

    //Publisher
    ros::Publisher serial_pub_;

    //Subscriber
    ros::Subscriber joy_msg_sub_;

    std::thread send_receive_thread;
    std::mutex encoder_mtx;

    void initValue(void);
    void initSubscriber(ros::NodeHandle& nh_);
    void initPublisher(void);

    bool serial_connect(void);
    void send_serial(void);
    void receive_serial(void);
    void send_receive_serial(void);
    void receive_encoder(void);
    void count_revolution(void);

    void set_val(float Linear_velocity,float angular_velocity);
    void get_val();

    unsigned char CalcChecksum(unsigned char* data, int leng);

    void joy_msg_callback(const sensor_msgs::Joy::ConstPtr &_joy_msg);
    void convert_cmd_vel();
    
public:
    int LeftEncoder, RightEncoder;

    void runLoop(void);

    Chic_m4k(ros::NodeHandle &_nh):
    nh_(_nh),toggle_button(0),linear(0),angular(0),Linear_velocity(velocity_zero),angular_velocity(velocity_zero),
    temp_LEncoder(-1),temp_REncoder(-1),LeftEncoder(0),RightEncoder(0)
    {
        initValue();
        initSubscriber(nh_);
        serial_connect();
        send_receive_serial();

        receive_encoder();

        //set_val(255.0f,0.0f);
    }

    ~Chic_m4k()
    {
        close(serial_port);
    }
};