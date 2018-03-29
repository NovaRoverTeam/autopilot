
#include "ros/ros.h"
#include <stdlib.h>

using namespace std;

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
