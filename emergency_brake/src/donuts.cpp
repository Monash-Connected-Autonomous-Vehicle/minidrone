#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"


int main(int argc, char **argv)
{

  // ros setup
  ros::init(argc, argv, "donuts");
  ros::NodeHandle nh;

  // logging message
  ROS_INFO("Initialised Control Node: Doing Donuts");
  
  // setup publisher
  ros::Publisher control_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);

  ros::Rate loop_rate(10);


  geometry_msgs::TwistStamped msg;

  // set linear and angular velocity 
  msg.twist.linear.x = 0.1; // 0.22 is max speed
  msg.twist.angular.z = -0.75;

  // continuously publish twist
  while (ros::ok())
  {

    // publish control message
    control_pub.publish(msg);


    ros::spinOnce();

  }


  return 0;
}
