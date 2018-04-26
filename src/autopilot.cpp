
#include "ros/ros.h"
#include "ros/console.h"
#include <stdlib.h>
#include <cmath>
#include <math.h> 

#include <gps/Gps.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Byte.h>
#include <autopilot/calc_route.h>
#include <autopilot/grid_size.h>
#include <rover/DriveCmd.h>

#define LOOP_HZ 10
#define MSG_PERIOD 5 // Period in seconds of message printing

using namespace std;

ros::NodeHandle* n;
ros::ServiceClient auto_client;
ros::ServiceClient grid_client;

typedef unsigned char byte;

struct latlng // GPS coordinate format
{
  float latitude;
  float longitude;
};

vector<latlng> route; // Route to be followed by autopilot
int des_wp; // Index of next waypoint to be approached
int n_wps; // Number of waypoints on the route

int bearing; // Current rover bearing
latlng rover_pos; // Current GPS lat/long of the rover
float lidar_angle; // Which way to go, according to LIDAR, relative

float lat_size; // Grid unit sizes in latitude/longitude format
float long_size;

float WP_THRES; // How close to wps until they're classed as reached
float LIDAR_TURN; // How much to turn by when LIDAR says there's object
float MAX_ANGLE; // How much to turn by when LIDAR says there's object
float MAX_SPD; // What percentage of max speed to do in auto mode?

int msg_cnt; // Counter for message printing

bool disable_lidar; // ROS parameter

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Setup:
//    General setup function for initialising variables.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void Setup()
{
  des_wp = 0; // Start at the first waypoint

  WP_THRES = 0.3; // Proportion of grid lat/long size req for wp arrival
  LIDAR_TURN = 45; // Try to turn 90 deg
  MAX_ANGLE = 90;
  MAX_SPD = 0.3; // 30% max speed

  msg_cnt = 0;

  autopilot::grid_size srv; // Create service message
  bool success = false;  

  while (!success)
  {    
    success = grid_client.call(srv);

    if (success) ROS_INFO("\n-- Successfully retrieved grid sizes."); 
    else ROS_INFO("\n-- Failed to retrieve grid sizes. Retrying..."); 

    ros::Duration(2).sleep(); 
  }

  lat_size = srv.response.lat_size; // Store the grid unit sizes
  long_size = srv.response.long_size;
}


float fclamp(float value, float max, float min)
{
  if (value > max) return max;
  else if (value < min) return min;
  else return value;
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Angle_Between:
//    Work out bearing between two GPS coordinates, deg clk from North.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
float Angle_Between(latlng coord1, latlng coord2)
{
  float lat1 = coord1.latitude; // convenient redefinitions
  float long1 = coord1.longitude;
  float lat2 = coord2.latitude;
  float long2 = coord2.longitude;

  float X = cos(lat1)*sin(lat2) 
            - sin(lat1)*cos(lat2)*cos(long2 - long1);

  float Y = cos(lat2)*sin(long2 - long1);

  float beta = atan2(Y, X);

  if (beta > 0) return beta; // Return pos clkwise deg from North
  else return 360 + beta;  
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Start_Auto:
//    Engages autonomous mode and requests the route.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
bool Start_Auto(autopilot::calc_route::Request  &req,
                autopilot::calc_route::Response &res)
{   
    // Possible states are STANDBY, TRAVERSE, AVOID, SEARCH
  n->setParam("/AUTO_STATE", "STANDBY"); // Auto starts in STANDBY

  ROS_INFO_STREAM("\nAttempting to start autopilot sequence.");

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

  ros::WallTime start_, end_; // Measure Glen execution time
  
  // Call the service Calc_Route, measuring time
  start_ = ros::WallTime::now();
  bool success = auto_client.call(srv);
  end_ = ros::WallTime::now();

  if (success)
  {
    ROS_INFO("\n-- Successfully retrieved waypoint route."); 

    double exe_time = (end_ - start_).toSec();
    ROS_INFO_STREAM("Glen execution time (s): " << exe_time << ".");
  }
  else 
    ROS_INFO("\n-- Failed to retrieve waypoint route."); 

  vector<gps::Gps> gps_route = srv.response.route;
  n_wps = route.size();

  // Populate the latlng route list from the Gps route list
  for (int i=0; i<n_wps; i++)
  {
    gps::Gps gps_coord = gps_route[i];

    latlng ll_coord;
    ll_coord.latitude = gps_coord.latitude;
    ll_coord.longitude = gps_coord.longitude;

    route.push_back(ll_coord);
  }

  n->setParam("/AUTO_STATE", "TRAVERSE"); // Start moving along route

  // Return the success of the service call to glen.py
  return success;
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Bearing_cb:
//    Callback for subscription to rover bearing, gets rover angle.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void Bearing_cb(const std_msgs::Int32::ConstPtr& msg)  
{   
  bearing = msg->data; // Set global bearing
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// LIDAR_cb:
//    Callback for subscription to LIDAR instructions.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void LIDAR_cb(const std_msgs::Byte::ConstPtr& msg)  
{   
  string AUTO_STATE; n->getParam("/AUTO_STATE", AUTO_STATE);

  if (AUTO_STATE != "STANDBY") // If we aren't being told to stop
  {
    byte raw = msg->data;
    bool bit2 = raw & 0b00000100; // Left third
    bool bit1 = raw & 0b00000010; // Middle third
    bool bit0 = raw & 0b00000001; // Right third

    if (bit1) // Only if there's something in the middle third, dodge
    {
      n->setParam("/AUTO_STATE", "AVOID"); // Start LIDAR avoidance state
    
      // If something LEFT, turn RIGHT
      if (bit2 && !bit0) lidar_angle = LIDAR_TURN;
      // If something RIGHT, turn LEFT
      else if (bit0 && !bit2) lidar_angle = -LIDAR_TURN;
      // If both left and right are occupied
      else 
      { // Turn in direction closest to next waypoint
        if (Angle_Between(rover_pos, route[des_wp]) - bearing > 0)
          lidar_angle = LIDAR_TURN;
        else
          lidar_angle = -LIDAR_TURN;
      }
    }
    else n->setParam("/AUTO_STATE", "TRAVERSE"); // Start LIDAR avoidance state
  }
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// GPS_cb:
//    Callback for subscription to current rover GPS coordinate.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void GPS_cb(const gps::Gps::ConstPtr& msg)  
{   
  rover_pos.latitude = msg->latitude;
  rover_pos.longitude = msg->longitude;
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Arrived:
//    Check if we have arrived at the destination location.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
bool Arrived(latlng coord1, latlng coord2)
{
  float delta_long = coord2.longitude - coord1.longitude;
  float delta_lat  = coord2.latitude  -  coord1.latitude;

  return (delta_long < long_size*WP_THRES) 
          && (delta_lat < lat_size*WP_THRES);
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main:
//    ROS setup and main control code for changing angle and moving
//    along the desired route.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot"); // Initialise ROS package
  n = new ros::NodeHandle("/");

  ros::Rate loop_rate(LOOP_HZ);	// Define loop rate

  ros::Subscriber bearing_sub = 
    n->subscribe("/bearing", 1, Bearing_cb);

  n->getParam("disable_lidar", disable_lidar);

  if (!disable_lidar)
  {
    ros::Subscriber LIDAR_sub = 
      n->subscribe("/shortRangeNav", 1, LIDAR_cb);
  }

  ros::Subscriber gps_sub = 
    n->subscribe("/gps/gps_data", 1, GPS_cb);

  ros::Publisher drive_pub = 
    n->advertise<rover::DriveCmd>("cmd_data", 1);

  ros::ServiceServer start_auto_srv = 
    n->advertiseService("Start_Auto", Start_Auto);

  auto_client = 
    n->serviceClient<autopilot::calc_route>("/Calc_Route");

  grid_client = 
    n->serviceClient<autopilot::grid_size>("/Grid_Size");

  // Possible states are STANDBY, TRAVERSE, AVOID, SEARCH
  n->setParam("/AUTO_STATE", "STANDBY"); // Auto starts in STANDBY

  Setup(); // Initialise vars and get grid sizes

  ROS_INFO_STREAM("Route has " << n_wps << " waypoints.");
 
  while (ros::ok())
  {
    string STATE; n->getParam("/STATE", STATE);
    string AUTO_STATE; n->getParam("/AUTO_STATE", AUTO_STATE);

    if ((msg_cnt > LOOP_HZ*MSG_PERIOD) && route.size() > 0)
    {
      ROS_INFO_STREAM("\nHeading towards wp #" << des_wp << ",");
      ROS_INFO_STREAM("  at  latitude  " << route[des_wp].latitude  << ",");
      ROS_INFO_STREAM("  and longitude " << route[des_wp].longitude << ".");

      ROS_INFO_STREAM("\nRelative angle to destination is " 
                      << Angle_Between(rover_pos, route[des_wp]));

      ROS_INFO_STREAM("\nDelta pos is lat  "  
                      << route[des_wp].latitude - rover_pos.latitude << ",");
      ROS_INFO_STREAM("             long " 
                      << route[des_wp].latitude - rover_pos.latitude << ".");

      msg_cnt = 0;
    }

    if (STATE == "AUTO")
    {
      if (AUTO_STATE == "AVOID") // LIDAR takes priority
      { 
        // Create ROS msg for drive command
        rover::DriveCmd msg;

        // Turn and/or drive
        msg.steer = fclamp(100*lidar_angle/MAX_ANGLE, 100.0, -100.0);
        msg.acc = copysign(100 - fabs(msg.steer), msg.steer);

        drive_pub.publish(msg); 
      }
      else if (AUTO_STATE == "TRAVERSE")
      {
        // If arrived at wp, set next wp as destination or start ball search
        if (Arrived(rover_pos, route[des_wp])) 
        {
          ROS_INFO_STREAM("\nArrived at wp #" << des_wp << "!");

          des_wp++; // Select next waypoint on route as new destination
   
          if (des_wp >= n_wps) // If we have arrived at the final waypoint
          {
            ROS_INFO("\nFinal waypoint reached! Beginning search for ball.");

            n->setParam("/AUTO_STATE", "SEARCH"); // Begin search for ball
          }
        }

        // Find angle we need to turn to get on course
        float des_angle = Angle_Between(rover_pos, route[des_wp]) - bearing;
      
        // Create ROS msg for drive command
        rover::DriveCmd msg;

        msg.steer = fclamp(100*des_angle/MAX_ANGLE, 100.0, -100.0);
        msg.acc = copysign(100 - fabs(msg.steer), msg.steer);

        drive_pub.publish(msg);         
      }       
    }

    msg_cnt++;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
