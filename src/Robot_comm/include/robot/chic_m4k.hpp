#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <thread>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define JOY_BUTTON_AMOUNT 12
#define JOY_AXES_AMOUNT 6

class Chic_m4k
{
private:
    typedef struct _Joy_msg
    {
        unsigned int seq;
        ros::Time stamp;
        std::string frame_id;

        float buttons[JOY_BUTTON_AMOUNT];
        float axes[JOY_AXES_AMOUNT];
    } Joy_msg;

    ros::NodeHandle nh_;
    
    int serial_port;
    std::string topic_name;
    Joy_msg joy_msg;

    //Publisher
    ros::Publisher serial_pub_;

    //Subscriber
    ros::Subscriber joy_msg_sub_;

    void initValue(void);
    void initSubscriber(ros::NodeHandle& nh_);
    void initPublisher(void);

    bool serial_connect(void);
    void send_serial(void);
    void receive_serial(void);
    void send_receive_serial(void);

    void joy_msg_callback(const sensor_msgs::Joy::ConstPtr &_joy_msg);
    
public:
    void runLoop(void);

    Chic_m4k(ros::NodeHandle &_nh):
    nh_(_nh)
    {
        initValue();
        initSubscriber(nh_);
        serial_connect();
    }
};