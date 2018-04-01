
#include "ros/ros.h"
#include <stdlib.h>

#include <gps/Gps.h>
#include <std_msgs/Float32.h>
#include <autopilot/calc_route.h>

#define LOOP_HZ 10

using namespace std;

ros::NodeHandle* n;
ros::ServiceClient auto_client;

vector<Gps> route;
float bearing;


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Start_Auto:
//    Engages autonomous mode and requests the route.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
bool Start_Auto(std_srvs::Trigger::Request  &req,
                std_srvs::Trigger::Response &res)
{   
  autopilot::calc_route srv; // Create service message

  // Forward the request to the Calc_Route service
  if (req.latlng = true)
  {
    srv.request.latlng = true;
    srv.request.destination = req.destination;
  }
  else
  {
    srv.request.latlng = false;
    srv.request.bearing = req.bearing;
    srv.request.distance = req.distance;
  }
  
  // Call the service Calc_Route, 
  auto_client.call(srv);

  // Return the success of that service call
  return srv.response.success;
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Bearing_cb:
//    Callback for subscription to rover bearing, gets rover angle.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void Bearing_cb(const std_msgs::Float32::ConstPtr& msg)  
{   
  bearing = msg->data; // Set global bearing
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// LIDAR_cb:
//    Callback for subscription to LIDAR instructions.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void LIDAR_cb(const std_msgs::Float32::ConstPtr& msg)  
{   
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot"); // Initialise ROS package
  n = new ros::NodeHandle();

  ros::Rate loop_rate(LOOP_HZ);	// Define loop rate

  ros::Subscriber bearing_sub = 
    (*n).subscribe("/imu/bearing", 1, Bearing_cb);

  ros::ServiceServer start_auto_srv = 
    (*n).advertiseService("Start_Auto", Start_Auto);

  auto_client = 
    (*n).serviceClient<autopilot::calc_route>("/Calc_Route");
 
  while (ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
