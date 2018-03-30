#!/usr/bin/env python

# *** THIS IS A TEST TO SEE IF THE CALC_ROUTE SERVICE WORKS
#
# run this with "rosrun autopilot test_client.py"
#   if this doesn't work, make sure it has execute permission
#     - go to the containing folder and run 'chmod a+x test_client.py'
#   also 'roscore' must be running

import sys
import rospy
from gps.msg import Gps # Import our custom ROS message for GPS data
from autopilot.srv import * # Import service request for route plan

# Client to utilise the calc_route service
def calc_route_client(gps_coord):

    rospy.wait_for_service('/Calc_Route')

    # Function handle for the service
    Calc_Route = rospy.ServiceProxy('/Calc_Route', calc_route)
    resp = Calc_Route(gps_coord) # Call the service
    return resp.route # Return the response of the service


def program():
    # Initialise the test client node
    rospy.init_node('test_client')
    rate = rospy.Rate(0.5) # Set the loop rate to 0.5hz

    while not rospy.is_shutdown(): 

      print "Trying to request route!"     

      gps_coord = Gps() # Create an arbitrary GPS coordinate
      gps_coord.latitude = -3.0  
      gps_coord.longitude = 14.0

      print gps_coord # testing, print coordinate

      route = calc_route_client(gps_coord)
      print route # testing, print resulting route

      rate.sleep() # Sleep to maintain the loop rate


if __name__ == "__main__":
    program()

