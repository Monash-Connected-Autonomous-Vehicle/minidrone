
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

class EmergencyBrake
{
  public:

  EmergencyBrake() 
  {

    // get stopping distance param
    ros::param::get("/stop_distance", stop_distance_);
    ros::param::get("/stop_time", stop_time_);
    ROS_INFO("Stopping Distance set at %f m", stop_distance_);
    // ROS_INFO("Stopping Time set at %f m", stop_time_);

    // laser subscriber
    laser_sub_ = nh_.subscribe("/scan", 10, &EmergencyBrake::LaserCallback, this);

    // odom subscriber
    odom_sub_ = nh_.subscribe("/odom", 10, &EmergencyBrake::OdomCallback, this);


    // control publisher
    control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);

  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {

    // setup twist message
    geometry_msgs::TwistStamped twist;
    twist.twist.angular.z = 0.0;


    if (BrakeTriggered_ != true) {

      for(int i=0;i<(msg->ranges.size());i++){
        //_lsval.ranges[i]=msg->ranges[i];

          if(!(msg->ranges.empty())) {

            // only check the scan directly infront of turtlebot 
            // layout is (0 - 360)
            // You probably want to take the average of a number of points infront of the vehicle...
            if (i == 0) { 
              
              // calculate time to collision
              float distance_to_object = msg->ranges[i];
              float time_to_collision = distance_to_object / vehicle_speed_;

              ROS_INFO("Distance to Object: %f m", msg->ranges[i]);
              ROS_INFO("Vehicle Speed : %f m/s", vehicle_speed_);
              ROS_INFO("Time to Collision: %f s \n", time_to_collision);
              // ROS_INFO("Time to Stop: %d s", stop_time_);

              if (time_to_collision <= stop_time_) {

                // crash imminent
                twist.twist.linear.x = 0.0;

                BrakeTriggered_ = true;

                ROS_INFO("Emergency Brake Triggered. Stopping Vehicle.");

              } else {

                // continue if emergency brake is not triggered
                  twist.twist.linear.x = 0.22;

              }
            }

          } else{

              ROS_INFO("No Laser Data Received...");

          }
      }

      // publish twist message
      control_pub_.publish(twist);
    }
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{

  //ROS_INFO("Vehicle Speed : %f m/s", msg->twist.twist.linear.x);

  // read vehicle speed
  vehicle_speed_ = msg->twist.twist.linear.x;

}


private:
  ros::NodeHandle nh_; 
  ros::Publisher control_pub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  

  float vehicle_speed_ = 0.0;
  double stop_distance_ = 1.0;
  float stop_time_ = 2.0;
  bool BrakeTriggered_ = false;

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "adv_emergency_brake");

  EmergencyBrake EmBrake;

  ros::spin();

  return 0;
}



// https://answers.ros.org/question/60239/how-to-extract-and-use-laser-data-from-scan-topic-in-ros-using-c/
// https://get-help.robotigniteacademy.com/t/how-to-read-the-laserscan-store-and-display-it-simultaneously-topics-quiz/990/7
// https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/