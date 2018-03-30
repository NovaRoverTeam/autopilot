#!/usr/bin/env python

# Template for the high level route planning node, which has a subscription
# to the GPS node, and hosts a service called "Calc_Route" for providing a 
# route to the autopilot.
#
# run this with "rosrun autopilot glen.py"
#   if this doesn't work, make sure it has execute permission
#     - go to the containing folder and run 'chmod a+x glen.py'
#   also 'roscore' must be running

import rospy
from gps.msg import Gps # Import our custom ROS message for GPS data
from autopilot.srv import * # Import service message for route plan


def Calc_Route(req):

    req.destination # Do something with the Gps coordinate of destination

    print req.destination # testing

    # Initialise list of route coords, the type must be the ROS msg Gps
    route = [] 

    # *** TODO GENERATE THE ROUTE HERE, see examples below ***

    response = calc_routeResponse() # Create the service response message

    gps_coord = Gps() # Create an individual GPS coordinate
    gps_coord.latitude = -555.555  # Set latitude and longitude to whatever
    gps_coord.longitude = 123.12345
    route.append(gps_coord) # Append the coordinate to the route list

    gps_coord = Gps() # Add another coordinate to list
    gps_coord.latitude = -666.666
    gps_coord.longitude = 123.45678
    route.append(gps_coord) 

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

