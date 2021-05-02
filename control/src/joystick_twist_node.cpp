#include <ros/ros.h>
#include "joystick_twist/joystick_twist.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_twist");
  JoystickTwist joystick_twist;

  ros::spin();
}
