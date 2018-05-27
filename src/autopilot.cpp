
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
#include <autopilot/set_dest.h>
#include <rover/DriveCmd.h>
#include <rover/Retrieve.h>

#define LOOP_HZ 10
#define MSG_PERIOD 5 // Period in seconds of message printing

using namespace std;

ros::NodeHandle* n;
ros::ServiceClient auto_client;
ros::ServiceClient grid_client;
ros::Subscriber LIDAR_sub; 

typedef unsigned char byte;

struct latlng // GPS coordinate format
{
  double latitude;
  double longitude;
};

vector<latlng> route; // Route to be followed by autopilot
int des_wp; // Index of next waypoint to be approached
int n_wps; // Number of waypoints on the route

int bearing; // Current rover bearing
latlng rover_pos; // Current GPS lat/long of the rover
double lidar_angle; // Which way to go, according to LIDAR, relative
bool lidar_reverse; // True if we need to back up

double grid_size; // Distance in metres of grid units
double lat_size; // Grid unit sizes in latitude/longitude format
double long_size;

double WP_THRES; // How close to wps until they're classed as reached
double LIDAR_TURN; // How much to turn by when LIDAR says there's object
double MAX_ANGLE; // How much to turn by when LIDAR says there's object
double MAX_SPD; // What percentage of max speed to do in auto mode?

int msg_cnt; // Counter for message printing

bool disable_lidar = 0; // ROS parameter
bool glen_enabled = 0;

latlng retrieval_dest; // Retrieval destination
bool retrieval_flag;

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Setup:
//    General setup function for initialising variables.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
void Setup()
{
  des_wp = 0; // Start at the first waypoint

  WP_THRES = 10; // Distance in metres that counts as a wp being reached
  LIDAR_TURN = 90; // Try to turn 90 deg
  MAX_ANGLE = 90;
  MAX_SPD = 0.3; // 30% max speed

  msg_cnt = 0;
  retrieval_flag = 0; // Set high to enable retrieval publishing

  autopilot::grid_size srv; // Create service message
  bool success = false;  

  while (!success)
  {    
    success = grid_client.call(srv);

    if (success) ROS_INFO("\n-- Successfully retrieved grid sizes."); 
    else ROS_INFO("\n-- Failed to retrieve grid sizes. Retrying..."); 

    ros::Duration(2).sleep(); 
  }

  //grid_size = srv.response.grid_size;
  grid_size = 10;
  lat_size = fabs(srv.response.lat_size); // Store the grid unit sizes
  long_size = fabs(srv.response.long_size);

  ROS_INFO_STREAM("lat size " << lat_size);
}


double fclamp(double value, double max, double min)
{
  if (value > max) return max;
  else if (value < min) return min;
  else return value;
}



//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Angle_Between:
//    Work out bearing between two GPS coordinates, deg clk from North.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
double Angle_Between(latlng coord1, latlng coord2)
{
  double lat1 = M_PI*coord1.latitude/180; // convenient redefinitions
  double long1 = M_PI*coord1.longitude/180;
  double lat2 = M_PI*coord2.latitude/180;
  double long2 = M_PI*coord2.longitude/180;

  double X = cos(lat1)*sin(lat2) 
            - sin(lat1)*cos(lat2)*cos(long2 - long1);

  double Y = cos(lat2)*sin(long2 - long1);

  double beta = 180*atan2(Y, X)/M_PI;

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
  bool success;

  ROS_INFO_STREAM("\nAttempting to start autopilot sequence.");

  autopilot::calc_route srv; // Create service message

  // Forward the request to the Calc_Route service
  if (req.latlng == true)
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

  glen_enabled = srv.request.glen_enabled;
  disable_lidar = srv.request.disable_lidar;
 
  ros::WallTime start_, end_; // Measure Glen execution time
  
  route.clear(); // Reset the route vector
  des_wp = 0;

 float actual_angle ;

  // Call the service Calc_Route, measuring time
  if(glen_enabled)
  {
    start_ = ros::WallTime::now();
    success = auto_client.call(srv);
    end_ = ros::WallTime::now();

    if (success)
    {
      ROS_INFO("\n-- Successfully retrieved waypoint route.");

      double exe_time = (end_ - start_).toSec();
      ROS_INFO_STREAM("Glen execution time (s): " << exe_time << ".");
      
      n->setParam("/STATE", "AUTO"); // Set auto state
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
  }
  else // If Glen disabled
  {
    latlng coord1;
    coord1.latitude   = rover_pos.latitude;
    coord1.longitude  = rover_pos.longitude;

    latlng coord2;
    if (req.latlng == true)
    {
      coord2.latitude   = req.destination.latitude;
      coord2.longitude  = req.destination.longitude;
    }
    else // bearing + distance format
    {
      actual_angle = (M_PI/180.0)*(90 + (360 - req.bearing));

      coord2.latitude  = rover_pos.latitude 
        + lat_size*(req.distance/grid_size)*sin(actual_angle);
      coord2.longitude = rover_pos.longitude 
        + long_size*(req.distance/grid_size)*cos(actual_angle);
    }
    
    // Simple interpolation
    for (int j=20; j<=100; j=j+20)
    {
      latlng ll_coord;
      ll_coord.latitude = coord1.latitude + (coord2.latitude - coord1.latitude) * (((float) j)/100);
      ll_coord.longitude = coord1.longitude + (coord2.longitude - coord1.longitude) * (((float) j)/100);
      route.push_back(ll_coord);

      success = 1;
    }

    n_wps = route.size();

    n->setParam("/STATE", "AUTO"); // Set auto state
  }

  ROS_INFO_STREAM("Route has " << n_wps << " waypoints.");

  n->setParam("/AUTO_STATE", "TRAVERSE"); // Start moving along route

  // Return the success of the service call to glen.py
  return success;
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// Set_Dest:
//    Set the destination for the interface to display.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
bool Set_Dest(autopilot::set_dest::Request  &req,
              autopilot::set_dest::Response &res)
{   
  ROS_INFO_STREAM("\nAttempting to begin destination publishing.");
  
  retrieval_flag = true; // Enable retrieval msg publishing

  retrieval_dest.latitude = req.destination.latitude;
  retrieval_dest.longitude = req.destination.longitude;

  return true;
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
  string STATE; n->getParam("/STATE", STATE);

  if (STATE != "STANDBY") // If we aren't being told to stop
  {
    byte raw = msg->data;
    bool bit3 = raw & 0b10001000; // Do we need to reverse?
    bool bit2 = raw & 0b01000100; // Left third
    bool bit1 = raw & 0b00100010; // Middle third
    bool bit0 = raw & 0b00010001; // Right third

    if (bit3) { // If something is too close 
      n->setParam("/AUTO_STATE", "AVOID");
      // If something LEFT, turn RIGHT
      if (bit2 && !bit0) lidar_angle = LIDAR_TURN;
      // If something RIGHT, turn LEFT
      else if (bit0 && !bit2) lidar_angle = -LIDAR_TURN;
      // If both left and right are occupied
      else
      { // Turn in direction closest to next waypoint
      if (Angle_Between(rover_pos, route[des_wp]) - bearing > 0) lidar_angle = LIDAR_TURN;
      else lidar_angle = -LIDAR_TURN;
      }
    }
    else if (STATE == "AVOID"){ // If something is already not too close
      n->setParam("/AUTO_STATE", "JUST_AVOIDED");
    }
    else if (STATE == "JUST_AVOIDED"){
      if (!bit1){ // If something has moved away from the center
        n->setParam("/AUTO_STATE", "TRAVERSE");
      }
    }
    //All gucci
    else n->setParam("/AUTO_STATE", "TRAVERSE");
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
bool Arrived(latlng coord1, latlng coord2, double *dist)
{
  double delta_long = coord2.longitude - coord1.longitude;
  double delta_lat  = coord2.latitude  -  coord1.latitude;

  // Calculate distance in metres to destination
  double distance = grid_size*sqrt(pow(delta_long/long_size, 2) 
                                  + pow(delta_lat/lat_size, 2));

  *(dist) = distance; // Set distance

  return distance < WP_THRES;
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main:
//    ROS setup and main control code for changing angle and moving
//    along the desired route.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot"); // Initialise ROS package
  n = new ros::NodeHandle();

  ros::Rate loop_rate(LOOP_HZ);	// Define loop rate
  
  ros::Subscriber bearing_sub = 
    n->subscribe("/bearing", 1, Bearing_cb);

  //n->getParam("disable_lidar", disable_lidar);
  //ROS_INFO_STREAM("disable_lidar " << disable_lidar);
   
  if (!disable_lidar)
    LIDAR_sub = n->subscribe("/shortRangeNav", 1, LIDAR_cb);

  ros::Subscriber gps_sub = 
    n->subscribe("/gps/gps_data", 1, GPS_cb);

  ros::Publisher drive_pub = 
    n->advertise<rover::DriveCmd>("cmd_data", 1);

  ros::Publisher dest_pub = 
    n->advertise<rover::Retrieve>("retrieve", 1);

  ros::ServiceServer start_auto_srv = 
    n->advertiseService("/Start_Auto", Start_Auto);

  ros::ServiceServer set_dest_srv = 
    n->advertiseService("/Set_Dest", Set_Dest);

  auto_client = 
    n->serviceClient<autopilot::calc_route>("/Calc_Route");

  grid_client = 
    n->serviceClient<autopilot::grid_size>("/Grid_Size");

  // Possible states are STANDBY, TRAVERSE, AVOID, SEARCH
  n->setParam("/AUTO_STATE", "STANDBY"); // Auto starts in STANDBY

  Setup(); // Initialise vars and get grid sizes

  int retrieve_time = 1*LOOP_HZ - 1;
  int retrieve_cnt = 0;

  while (ros::ok())
  {
    string STATE; n->getParam("/STATE", STATE);
    string AUTO_STATE; n->getParam("/AUTO_STATE", AUTO_STATE);
    double proximity; // Distance to wp, for debugging 

    ROS_INFO_STREAM("State is " << STATE << ", autostate is " << AUTO_STATE);

    if (STATE == "AUTO")
    {  
        if (msg_cnt > LOOP_HZ*MSG_PERIOD)  
        {
          ROS_INFO_STREAM("AUTO_STATE is " << AUTO_STATE);
          ROS_INFO_STREAM("\nHeading towards wp #" << des_wp 
                      << ",\n  at  latitude  " << route[des_wp].latitude
                      << ",\n  and longitude " << route[des_wp].longitude << ".");

          ROS_INFO_STREAM("\nRelative angle to destination is " 
                      << Angle_Between(rover_pos, route[des_wp]) - bearing
                      << "\nProximity is " << proximity << " metres.");

          ROS_INFO_STREAM("\nDelta pos is lat  "  
                      << route[des_wp].latitude - rover_pos.latitude
                      << ",\n             long " 
                      << route[des_wp].latitude - rover_pos.latitude 
                      << ".");

          msg_cnt = 0;
        }

   
	if (AUTO_STATE == "AVOID") // LIDAR takes priority
	{
	  // Create ROS msg for drive command
	  rover::DriveCmd msg;

	  // Turn and/or drive
	  msg.steer = fclamp(100*lidar_angle/MAX_ANGLE, 100.0, -100.0);
	  msg.acc = 0;

	  drive_pub.publish(msg); 
	}
	else if (AUTO_STATE == "JUST_AVOIDED")
	{
	    // Create ROS msg for drive command
	  rover::DriveCmd msg;

	  // Turn and/or drive
	  msg.steer = 0;
	  msg.acc =  50;

	  drive_pub.publish(msg); 
	}
	else if (AUTO_STATE == "TRAVERSE")
	{ 
	  // If arrived at wp, set next wp as destination or start ball search
	  if (Arrived(rover_pos, route[des_wp], &proximity)) 
	  {
	    ROS_INFO_STREAM("\nArrived at wp #" << des_wp << "!");

	    des_wp++; // Select next waypoint on route as new destination

	    if (des_wp >= n_wps) // If we have arrived at the final waypoint
	    {
	      ROS_INFO("\nFinal waypoint reached! Beginning search for ball.");

              n->setParam("/AUTO_STATE", "SEARCH"); // Begin search for ball
            }	  
          }

          // Take bearing to 0 degrees
          double bearing_to_360 = 360 - bearing;

	  // Find difference between 0 degrees and the destination angle after
          //   shifting it by the same amount as the bearing
	  double des_angle_raw = Angle_Between(rover_pos, route[des_wp])
                                 + bearing_to_360;

          // Make sure the value is in the range 0 to 360
          if (des_angle_raw > 360) des_angle_raw -= 360;

          // The desired relative angle to change by
          double des_angle = 0; 

          // Find relative angle
          if (des_angle_raw > 180) des_angle = -(360 - des_angle_raw);
          else                     des_angle = des_angle_raw;

          ROS_INFO_STREAM("des_angle is " << des_angle);

	  // Create ROS msg for drive command
	  rover::DriveCmd msg;
	  
	  msg.steer = fclamp(100*des_angle/MAX_ANGLE, 100.0, -100.0);
	  msg.acc = 100 - fabs(msg.steer);

	  drive_pub.publish(msg); ROS_INFO("pubbing auto cmd");         
	}       
         
    }
    else n->setParam("/AUTO_STATE", "STANDBY");

    if (retrieval_flag && retrieve_cnt > retrieve_time)
    {
      rover::Retrieve msg;
      msg.bearing = Angle_Between(rover_pos, retrieval_dest);

      double dist;
      Arrived(rover_pos, retrieval_dest, &dist);
      msg.distance = dist;

      dest_pub.publish(msg);
      retrieve_cnt = 0;
    }

    retrieve_cnt++;
    msg_cnt++;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
