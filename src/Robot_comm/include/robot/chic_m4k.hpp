#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#define JOY_BUTTON_AMOUNT 12
#define JOY_AXES_AMOUNT 6

class Chic_m4k
{
private:
    struct Joy_msg
    {
        unsigned int seq;
        ros::Time stamp;
        std::string frame_id;

        float buttons[JOY_BUTTON_AMOUNT];
        float axes[JOY_AXES_AMOUNT];
    };

    ros::NodeHandle nh_;
    sensor_msgs::Joy joy_msg;
    std::string topic_name;

    //Publisher
    ros::Publisher serial_pub_;

    //Subscriber
    ros::Subscriber joy_msg_sub_;

    void initValue(void);
    void initSubscriber(void);
    void initPublisher(void);

    void joy_msg_callback(const sensor_msgs::Joy::ConstPtr &_joy_msg);
    
public:
    void runLoop(void);

    Chic_m4k(ros::NodeHandle _nh):
    nh_(_nh)
    {
        void initValue(void);
        void initSubscriber(void);
        void initPublisher(void);
    }
};