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

# NOTE: verify all imports are requried
import cv2
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from math import radians, sin, cos, sqrt, asin, degrees, atan, floor

# Global variables 
currentRoverLat = 0
currentRoverLong = 0
mapLoadFlag = 0

#Other functions*****************************************************************************************
class MapDetails:# =================================================================================

    def Haversine(planetRadius, lat1, long1, lat2, long2):
        """Calculates the distance inbetween two GPS coordinates on a planet"""
	dLat = radians(lat2 - lat1)
	dLon = radians(long2 - long1)
	lat1 = radians(lat1)
	lat2 = radians(lat2)

	a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
	c = 2*asin(sqrt(a))
	return planetRadius * c

    def TestMapDimensionInfo(mapImg, planet, lat1, long1, lat2, long2): #not currently used because had coded below
        """Checks the dimesntions of a graph and returns the incremental change in the latitude/latitude"""
        if(planet == 1):#1=earth, 0=mars
            planetRadius = 6378137 #earth
        else:
            planetRadius = 3396200 #mars
        # count number of pixels from current location to find distance N, S, E, W etc.
        latDiff = lat1 - lat2
        longDiff = long1 - long2
        latNumCoords = mapImg.shape[0]
        longNumCoords = mapImg.shape[1]
        pixelIncrementLong = longDiff/longNumCoords
        pixelIncrementLat = latDiff/latNumCoords

        newCoordDistLat = lat1 + pixelIncrementLat
        newCoordDistLong = long1 + pixelIncrementLong

        latitude1 = 0
        longitude2 = long1
        latitude2 = pixelIncrementLat
        longitude1 = long1
        distance = MapDetails.Haversine(planetRadius, latitude1, longitude1, latitude2, longitude2)
        #print("Distance on Planet for pixel hight", distance)

        longitude2 = pixelIncrementLong
        latitude2 = lat1
        latitude1 = lat1
        longitude1 = 0
        distance = MapDetails.Haversine(planetRadius,latitude1, longitude1, latitude2, longitude2)
        #print("Distance on Planet for pixel width", distance)

        longitude2 = long1
        longitude1 = long1
        latitude1 = lat1
        latitude2 = lat2
        distance = MapDetails.Haversine(planetRadius,latitude1, longitude1, latitude2, longitude2)
        #print("Distance on Planet for hight of map", distance)

        latitude2 = lat1
        latitude1 = lat1
        longitude1 = long1
        longitude2 = long2
        distance = MapDetails.Haversine(planetRadius,latitude1, longitude1, latitude2, longitude2)
        #print("Distance on Planet for width of map", distance)
        return pixelIncrementLat, pixelIncrementLong
#end class MapDetails -------------------------------------------------------------------------------------

class PrintRoute: # ===========================================================================================
    @staticmethod
    def PrintRouteGraph(g, routeLat1, routeLong1, routeLat2, routeLong2):
        """Printing route will be included with zoomable visualisation, for now unused sections have been removed/
        commented. Used to determine the route"""
        path=nx.shortest_path(g) # source,target not specified
        route = path[(routeLat1,routeLong1)][(routeLat2,routeLong2)]
        return route #, subgraphColours
    @staticmethod
    def PrintGPSRoute(graph, desiredRoute): # Round GPS coordinates to 6 decimal places
        #print(nx.get_node_attributes(g, 'gpsCoordinate'))
        routeGPSCoord = []
        for i in range(len(desiredRoute)):
            routeGPSCoord.append(graph.nodes[desiredRoute[i]]['gpsCoordinate'])
        return routeGPSCoord
    @staticmethod 
    def FindNodeFromGPS(nodeLat, nodeLong, latBoundaryTL, longBoundaryTL,latBoundaryBR, longBoundaryBR, nodeGPSIncrementLat, nodeGPSIncrementLong):
        """Finds the nearest node label from a GPS coordinate, given the graph/map GPS boundaries"""
        #check nodeLat and nodeLong are within the range of the graph 
        if(nodeLat < latBoundaryTL and nodeLat > latBoundaryBR):
            rospy.loginfo("The GPS latitude recived is outside of the graph range")
            # set to the nearest value - take the most direct route to this node
        if(nodeLat < longBoundaryTL and nodeLat > longBoundaryBR):
            rospy.loginfo("The GPS longitude recived is outside of the graph range")
            # set to the nearest value - take the most direct route to this node
            
        #print('node lat:',nodeLat)
        #print('latBoundaryTL:',latBoundaryTL)
        #print('nodeGPSIncrementLat:',nodeGPSIncrementLat)
        #print('nodeGPSIncrementLong:',nodeGPSIncrementLong)
        unroundedNodeLat =   (nodeLat - latBoundaryTL) /  nodeGPSIncrementLat
        unroundedNodeLong =  (nodeLong - longBoundaryTL) / nodeGPSIncrementLong
        #print('unroundedNodeLat',unroundedNodeLat)
        nodeNameX = round(unroundedNodeLat)
        nodeNameY = round(unroundedNodeLong)
        scalingFactorLat = unroundedNodeLat - nodeLat #Direction included
        scalingFactorLong = unroundedNodeLong - nodeLong
        #print('scalingFactorLat:', scalingFactorLat)
        
        #attempt to find the nearest route if GPS coordinate out of range- direct link in the required direciton then call a route

        return nodeNameX, nodeNameY,scalingFactorLat, scalingFactorLong  #returns the corresponding node

    def CallRoute(g, routeLat1, routeLong1, routeLat2, routeLong2):
        # call route through graph 
        # Route Lat and Long are currently node coordinates 
       
        #route, subgraphColours = PrintRoute.PrintRouteGraph(g, routeLat1, routeLong1, routeLat2, routeLong2)
        route = PrintRoute.PrintRouteGraph(g, routeLat1, routeLong1, routeLat2, routeLong2)
        gpsCoordRoute = nx.get_node_attributes(g,'gpsCoordinate')
        #print(nx.get_node_attributes(g, 'gpsCoordinate'))

        routeGPS = PrintRoute.PrintGPSRoute(g, route)
        #print('-----------')
        #print(routeGPS)
        routeGPSList = []
        for i in range(len(routeGPS)):
            for j in range(len(routeGPS[i])):
                #routeGPS[i][j] =  Decimal(round(routeGPS[i][j], 6))
                routeGPSList.append(round(routeGPS[i][j], 6))
                #print(routeGPS[i][j])
        #print('GPS Coordinates of Rotue:\n',routeGPSList)
        return routeGPSList
    #end class PrintRoute ---------------------------------------------------------------------------------
    
class ConvertDist2GPSCoord:# =================================================================================
    global currentRoverLat
    global currentRoverLong

    def DeltaGPSCoord(bearing):     
        deltaLatitude = distance*cos(bearing)  # angle taken from the y axis 
        deltaLongitude = distance*sin(bearing)
        #print("deltaLongitude:",deltaLongitude )
        #convert distance into GPS coordinates 
        deltaLatitudeGPS = deltaLatitude/distPerDegLat
        deltaLongitudeGPS = deltaLongitude/distPerDegLong
        #print("detaLatitudeGPS:", detaLatitudeGPS)
        #print("detaLongitudeGPS:",detaLongitudeGPS)
        return deltaLatitudeGPS, deltaLongitudeGPS

    def DetermineDestGPSCoord(bearing, distance, currentRoverLat, currentRoverLong): 
        ##NOTE: Calc from distance to GPS coord pair changes depending on locaiton on earth. May need to alter approximation for 
        # Utah - https://en.wikipedia.org/wiki/Decimal_degrees 
        distPerDegLat =  110574 #m/degree
        distPerDegLong = (111320*cos(radians(currentRoverLat))) #m/degree
        
        if(bearing>=0 and bearing<=90): #quadreant 1 #NOTE: distance of conversion is in meters
            deltaLatitude, deltaLongitude = DeltaGPSCoord(bearing)
            destinationLat = currentRoverLat + deltaLatitude
            destinationLong = currentRoverLong + deltaLongitude
            # convert distance in meters to a longitude/latitude 
            
        elif(bearing <= 180 and bearing >90): # quadrant 4 
            modifiedAngle = bearing - 90
            deltaLatitude, deltaLongitude = DeltaGPSCoord(bearing)
            destinationLat = currentRoverLat - deltaLatitude
            destinationLong = currentRoverLong + deltaLongitude
            
        elif(bearing <= 270 and bearing>180): #quadrant 3 
            modifiedAngle = bearing - 180
            deltaLatitude, deltaLongitude = DeltaGPSCoord(bearing)
            destinationLat = currentRoverLat - deltaLatitude
            destinationLong = currentRoverLong - deltaLongitude
            
        elif(bearing <360 and bearing >270): #quadent 2
            modifiedAngle = bearing - 270
            deltaLatitude, deltaLongitude = DeltaGPSCoord(bearing)
            destinationLat = currentRoverLat + deltaLatitude
            destinationLong = currentRoverLong - deltaLongitude
            
        else:
            rospy.loginfo("ERROR: The compass input out of range and must be inbetween 0-360 degrees!")
            
        return destinationLat, destinationLong
    #end class ConvertDist2GPSCoord ---------------------------------------------------------------------------------

#end other functions *************************************************************************************
def Calc_Route(req):
    global mapLoadFlag
    global currentRoverLat
    global currentRoverLong

    rospy.loginfo("Calc_Route service has been called ~ Glen")

    # load in map
    if(mapLoadFlag ==0):  
        monashGraph = nx.read_gpickle("/home/nova/catkin_ws/src/autopilot/scripts/monashGraph.gpickle") #load in pre-processed graph, must be in same directory
	rospy.loginfo("Graph has been loaded ~ Glen")
        mapLoadFlag = 1
    
    # Destination can be GPS or a distance + true bearing
    if(req.latlng == False): #HELPPP: how has ben flagged this????
        # convert distance to a GPS coordinate
        destRoverLat, destRoverLong = ConvertDist2GPSCoord.DetermineDestGPSCoord(req.bearing, req.distance, currentRoverLat, currentRoverLong) 
        rospy.loginfo("Find destination from direction and baring ~ Glen")
        # set flag to FlagDestGPS to True
        req.latlng = True
        
    if(req.latlng== True): # call route through graph 
        # find the nodes which correspond the current and destination GPS coordinates
        destRoverLat = req.destination.latitude # Do something with the Gps coordinate of destination 
        destRoverLong = req.destination.longitude
        rospy.loginfo("Longitude and latitude obtained from GPS node ~ Glen")
        

        #print(req.destination) # testing
        #lat1Farm = -36.570556 #values used to select a smalller region in DEM file
        #lat2Farm = -36.557778
        #long2Farm = 146.629722 
        #long1Farm = 146.600278
    
        lat1MonashSelected = -37.910108 # GPS coordianates for Monash ovals
        lat2MonashSelected = -37.914679
        long1MonashSelected = 145.134746 
        long2MonashSelected = 145.139853
        if(currentRoverLat < lat1MonashSelected and currentRoverLat>lat2MonashSelected ):
            rospy.loginfo("Latitude out of map range ~ Glen")
        if(currentRoverLong < long1MonashSelected and currentRoverLong>long2MonashSelected ):
            rospy.loginfo("Longitude out of map range ~ Glen")
        else:
            pixelIncrementLat = -6.620725388603314e-05 # hardcodedd values found by calling TestMapDimensionsInfo 
            pixelIncrementLong = 8.294084507037695e-05
            rospy.loginfo("Current Rover Latitude: %f",currentRoverLat)
            rospy.loginfo("Current Rover Longitude: %f",currentRoverLong)
            
            currentNodeLat, currentNodeLong, currentDistScalingFactorLat, currentDistScalingFactorLong = PrintRoute.FindNodeFromGPS(currentRoverLat, currentRoverLong, lat1MonashSelected, long1MonashSelected,lat2MonashSelected, long2MonashSelected, pixelIncrementLat, pixelIncrementLong)
            rospy.loginfo("Graph node found for from the rover's GPS coordinate ~ Glen")
            destNodeLat, destNodeLong, destDistScalingFactorLat, destDistScalingFactorLong = PrintRoute.FindNodeFromGPS(destRoverLat, destRoverLong, lat1MonashSelected, long1MonashSelected,lat2MonashSelected, long2MonashSelected, pixelIncrementLat, pixelIncrementLong)
            rospy.loginfo("Longitude and latitude obtained from GPS node ~ Glen")
            # find route through graph
            gpsRoute = PrintRoute.CallRoute(monashGraph, currentNodeLat, currentNodeLong, destNodeLat, destNodeLong)
            rospy.loginfo("Route called inbetween graph nodes ~ Glen")
            # Initialise list of route coords, the type must be the ROS msg Gps
            route = [] 

            # *** GENERATE THE ROUTE HERE***

            response = calc_routeResponse() # Create the service response message
            
            for i in range(0,len(gpsRoute),2):
                gps_coord = Gps() # Create an individual GPS coordinate
                gps_coord.latitude = gpsRoute[i]  # Set latitude and longitude to whatever
                gps_coord.longitude = gpsRoute[i+1]
                route.append(gps_coord) # Append the coordinate to the route list 

            response.route = route # Give route list to the service response
    
    return response # Return the response to the service request - the route.

def Grid_Size(req):

    response = grid_sizeResponse() # Create the service response message

    response.lat_size = -6.620725388603314e-05 #hard coded in from processing map
    response.long_size = 8.294084507037695e-05
    rospy.loginfo("Latitude and longitude size (difference inbetween nodes) returned~ Glen")
    return response # Return the response to the service request


def GPS_Callback(gps_msg):
    global currentRoverLat
    global currentRoverLong

    # This callback function runs whenever the GPS node publishes the 
    # GPS coordinate location of the rover (frequently).
    currentRoverLat = gps_msg.latitude  # Current latitude
    currentRoverLong = gps_msg.longitude # Current longitude
    

def glen():
    # Initialise the ROS node "glen", for high-level route planning
    rospy.init_node('glen')
    rate = rospy.Rate(10) # Set the loop rate to 10hz

    # Subscribes to the GPS_data topic.
    rospy.Subscriber("/gps/gps_data", Gps, GPS_Callback)

    # Sets up the Grid_Size service, to find lat and long distance between grid units.
    serv2 = rospy.Service('Grid_Size', grid_size, Grid_Size)

    # Sets up the Calc_Route service, to generate the GPS route.
    serv1 = rospy.Service('Calc_Route', calc_route, Calc_Route)

    

    while not rospy.is_shutdown():      
      # TODO anything that needs to run continuously, put in here
      
      rate.sleep() # Sleep to maintain the loop rate


if __name__ == '__main__':
    glen()

