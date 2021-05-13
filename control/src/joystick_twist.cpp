#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "joystick_twist/joystick_twist.h"

JoystickTwist::JoystickTwist():
  linear_axis_(1), // default to 1 (DualShock 4)
  angular_axis_(3), // default to 3 (DualShock 4)
  nh_("~")
{

  nh_.param("axis_linear", linear_axis_, linear_axis_);
  nh_.param("axis_angular", angular_axis_, angular_axis_);
  nh_.param("scale_angular", angular_scale_, angular_scale_);
  nh_.param("scale_linear", linear_scale_, linear_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/joy_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &JoystickTwist::joyCallback, this);
}

void JoystickTwist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist; // default initialises values to zero
  // twist.header.stamp = ros::Time::now();
  twist.angular.z = angular_scale_*joy->axes[angular_axis_];
  twist.linear.x = linear_scale_*joy->axes[linear_axis_];
  twist_pub_.publish(twist);

  // if square pressed, raise max speed
  // not sure which axes is buttons
  if (joy->buttons[1] == 1) {
    nh_.setParam("/mini_max_speed_pwm", 40);
  }

  // reset speed if circle pressed
  if (joy->buttons[2] == 1) {
    nh_.setParam("/mini_max_speed_pwm", 90);
  }


}
