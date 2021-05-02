#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class JoystickTwist
{
public:
    JoystickTwist();
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_axis_, angular_axis_;
    double linear_scale_, angular_scale_;
    ros::Publisher twist_pub_;
    ros::Subscriber joy_sub_;
};
