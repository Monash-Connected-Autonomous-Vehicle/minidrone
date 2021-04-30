
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class EmergencyBrake
{
  public:

  EmergencyBrake() 
  {

    // get stopping distance param
    ros::param::get("/stop_distance", stop_distance_);
    ros::param::get("/stop_time", stop_time_);
    ros::param::get("/field_of_view", IDX_RANGE_);

    ROS_INFO("Stopping Distance set at %.2f m", stop_distance_);
    ROS_INFO("Field of View set at %i deg.", 2*IDX_RANGE_);
    ROS_INFO("Stopping Time set at %.2f s", stop_time_);
    
    // laser subscriber
    laser_sub_ = nh_.subscribe("/scan", 10, &EmergencyBrake::LaserCallback, this);

    // control publisher
    control_pub_ = nh_.advertise<geometry_msgs::Twist>("/emergency_vel", 10);

    // laser publisher
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 10);



  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {

    // get stopping distance param
    ros::param::get("/stop_distance", stop_distance_);
    ros::param::get("/stop_time", stop_time_);
    ros::param::get("/field_of_view", IDX_RANGE_);

    // setup twist message
    geometry_msgs::Twist twist;
    twist.angular.z = 0.0;

    // setup laser scan message (filtered)
    sensor_msgs::LaserScan scan;
    scan = *msg;

    // if(!(msg->ranges.empty())) {
    // } else{
    // 
    // ROS_INFO("No Laser Data Received...");
    // 
    // }
  
    bool BrakeTriggered_ = false;

    for(int i=0; i < (msg->ranges.size()); i++){
      
      // TODO: make wider field of view
      // TODO: republish filtered points
      // TODO: make detection more robust
      // TODO: use cmd_vel speed to set TTC

      // check if scan index is in front of vehicle (0-15, 690 - 720)
      if (((i > MIN_IDX) && (i < IDX_RANGE_)) || ((i < MAX_IDX) && (i > MAX_IDX - IDX_RANGE_))) {


          // calculate time to collision
          // float distance_to_object = msg->ranges[i];
          // float time_to_collision = distance_to_object / vehicle_speed_;
          // TODO: odometry. otherwise this might be too circular

          // if object within range of any of the scans, stop the vehicle
          if (msg->ranges[i] < stop_distance_) {

            // object detected at close range, stop the vehicle
            ROS_INFO("Stopping the vehicle... %f m to object at %i deg.", msg->ranges[i], i/2);
            twist.linear.x = 0.0;
            BrakeTriggered_ = true;

          } 

      } else {
        
        // filter out points not in field of view
        scan.ranges[i] = 0;
      }
    }

    // no object detected at close range, continue driving
    if (BrakeTriggered_ != true) {
      ROS_INFO("Continue Driving... %f m to object.", msg->ranges[0]);
      //twist.twist.linear.x = 0.1;
      //twist.twist.angular.z = -0.75;
    } else {
      // publish twist message
      control_pub_.publish(twist);
    }

    // publish filtered scan 
    laser_pub_.publish(scan);
}

private:
  ros::NodeHandle nh_; 
  ros::Publisher control_pub_;
  ros::Subscriber laser_sub_;
  ros::Publisher laser_pub_;

  double stop_distance_ = 1.0;
  double stop_time_ = 1.0;
  const int MIN_IDX = 0;
  const int MAX_IDX = 720;
  int IDX_RANGE_ = 30;

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "emergency_brake");

  EmergencyBrake EmBrake;

  ros::spin();

  return 0;
}



// https://answers.ros.org/question/60239/how-to-extract-and-use-laser-data-from-scan-topic-in-ros-using-c/
// https://get-help.robotigniteacademy.com/t/how-to-read-the-laserscan-store-and-display-it-simultaneously-topics-quiz/990/7
// https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
