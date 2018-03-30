
#include "ros/ros.h"
#include <stdlib.h>

#include <std_msgs/Float32.h>

using namespace std;

void Glen_cb(const std_msgs::Float32::ConstPtr& msg)  
{   
  
}

void LIDAR_cb(const std_msgs::Float32::ConstPtr& msg)  
{   
  
}

void Sanity_cb(const std_msgs::Float32::ConstPtr& msg)  
{   
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot"); // Initialise ROS package
  ros::NodeHandle n;
  ros::Rate loop_rate(10);	// Define loop rate

  while (ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
