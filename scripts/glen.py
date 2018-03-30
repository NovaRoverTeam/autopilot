#!/usr/bin/env python
import rospy
from gps.msg import Gps # Import our custom ROS message for GPS data
from autopilot.srv import calc_route # Import service msg for route plan


def Calc_Route(req):

    req.destination # Do something with the Gps coordinate of destination

    # Initialise list of route coords, the type must be the ROS msg Gps
    route = [] 

    # *** TODO GENERATE THE ROUTE HERE, see examples below ***

    response = calc_routeResponse() # Create the service response message

    gps_coord = Gps() # Create an individual GPS coordinate
    gps_coord.latitude = -37.123432  # Set latitude and longitude to whatever
    gps_coord.longitude = 144.572783

    route.append(gps_coord) # Append the coordinate to the route list

    response.route = route # Give route list to the service response

    # *** FINISH GENERATING THE ROUTE HERE ***

    return response # Return the response to the service request - the route.


def GPS_Callback(gps_msg):
    # This callback function runs whenever the GPS node publishes the 
    # GPS coordinate location of the rover (frequently).

    # TODO erhaps store these in global variables? Do something with them
    gps_msg.latitude  # Current latitude
    gps_msg.longitude # Current longitude
    

def glen():
    # Initialise the ROS node "glen", for high-level route planning
    rospy.init_node('glen')
    rate = rospy.Rate(10) # Set the loop rate to 10hz

    # Subscribes to the GPS_data topic.
    rospy.Subscriber("/gps/gps_data", Gps, GPS_Callback)

    # Sets up the Calc_Route service, to generate the GPS route.
    serv = rospy.Service('Calc_Route', calc_route, Calc_Route)

    while not rospy.is_shutdown():      
      # TODO anything that needs to run continuously, put in here

      rate.sleep() # Sleep to maintain the loop rate


if __name__ == '__main__':
    glen()

