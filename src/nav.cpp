#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <ros/console.h>
#include "point.h"

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Byte.h"

using namespace std;

// number of range data points = 683

#define NUM_OF_SCAN_POINTS 683
#define MIN_RHO 0.07
#define SEPARATION 0.2
#define VAR_THRES 0.01
#define CORNER_ANGLE 1.0
#define MIN_PTS 5
#define BOTTOM_BOUND -0.2
#define UPPER_BOUND 3.2
#define FLOORLINE_BOTTOM_BOUND 1.0
#define FLOORLINE_UPPER_BOUND 2.0
#define SIDE_BOUND_FROM_CENTER 2.5
#define MIN_LENGTH 0.4
#define MIN_OBJECT_DISTANCE 0.2
#define OBSTACLE_DISTANCE 1.0
#define STOP 7
#define WINDOW 1


float angle_min, angle_increment;
bool dataReady;
vector<Point> pts;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	angle_min = scan->angle_min;
	angle_increment = scan->angle_increment;

	for(int i = 0; i < NUM_OF_SCAN_POINTS; i++) {
		float scan_data = scan->ranges[i];
		if (scan_data > MIN_RHO & !isnan(scan_data)){
			pts.push_back(Point(i,angle_min + i * angle_increment, scan_data));
		}
	}
	dataReady = true;
}

vector<Segment> cluster(const vector<Point>& pts){

	vector<Point> temp;
	vector<Point>::const_iterator it0 = pts.begin(), it1 = pts.begin()+1;
	vector<Segment> clusters, clusters0;

	while(it0 < pts.end() & it1 < pts.end()){
		Point pt0 = *(it1 - 1), pt1 = *it1;

		float a = pt1 - pt0;

		bool close = a <= SEPARATION;

		if (close){
			it1++;
		} else {
			temp.assign(it0,it1);
			clusters0.push_back(Segment(temp));
			it0 = it1 + 1;
			it1 = it0 + 1;
		}
	}

	temp.assign(it0,it1);
	clusters0.push_back(Segment(temp));

	for (unsigned int i = 0; i < clusters0.size(); i++){
		clusters.push_back(clusters0[i]);
	}

	return clusters;
}

vector<Segment> split(const Segment cluster){

	const vector<Point> clusterPt = cluster.pts;

	unsigned int i=0;

	vector<Segment> segments;
	vector<Point> firstPts, secondPts;
	vector<Point>::const_iterator b = clusterPt.begin(), it = clusterPt.begin()+WINDOW, e = clusterPt.end();
	float sumVar;
	vector<float> vars;

	while (it < (e - WINDOW)){

		firstPts.assign(b,it);
		secondPts.assign(it,e);

		Segment firstSeg = Segment(firstPts), secondSeg = Segment(secondPts);

		sumVar = firstSeg.var(true) + secondSeg.var(true);
		if (!isnan(sumVar)){
			vars.push_back(sumVar);
		} else {
			vars.push_back(1);
		}
		it++;
	}

	vector<float>::iterator result = min_element(vars.begin(), vars.end());

	it = clusterPt.begin() + distance(vars.begin(), result) + 1;

	firstPts.assign(b,it);
	secondPts.assign(it,e);

	Segment firstSeg = Segment(firstPts), secondSeg = Segment(secondPts);

	segments.push_back(firstSeg);
	segments.push_back(secondSeg);

	return segments;
}

void recursiveSplit(Segment cluster, vector<Segment>* list){
	if (cluster.var(true) <= VAR_THRES) list->push_back(cluster);
	else {
		vector<Segment> twoSegs = split(cluster);

		recursiveSplit(twoSegs[0], list);
		recursiveSplit(twoSegs[1], list);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "short_range_nav");
	ros::NodeHandle n;
	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
	ros::Publisher processPub=n.advertise<sensor_msgs::LaserScan>("/processScan",1);
	ros::Publisher navPub=n.advertise<std_msgs::Byte>("/shortRangeNav",1);

	while(ros::ok())
	{
		if (dataReady){

			sensor_msgs::LaserScan processed;
			std_msgs::Byte nav;
			vector<Segment>::iterator it;

			int smallObjectDetected = 0, largeObjectDetected = 0;

			//ROS_INFO("%u",pts.size());
			vector<Segment> clusters = cluster(pts);

			vector<Segment> listOfSegments0, listOfSegments;

			for (it = clusters.begin() ; it != clusters.end(); ++it){
				recursiveSplit(*it, &listOfSegments0);
			}

			for (it = listOfSegments0.begin() ; it != listOfSegments0.end(); ++it){
				if (it->pts.size() > MIN_PTS) listOfSegments.push_back(*it);
			}

			for (it = listOfSegments.begin() ; it != listOfSegments.end(); ++it){
				vector<float> xy = it->centerPt();
				float major = it->var(false), length = it->length();
				//ROS_INFO("%f", length/major);
				//
			}


			ros::Time scan_time = ros::Time::now();
			//populate the LaserScan message
			processed.header.stamp = scan_time;
			processed.header.frame_id = "laser";
			processed.angle_min = -2.09234976768;
			processed.angle_max = 2.09234976768;
			processed.angle_increment = 0.00613592332229;
			processed.time_increment = 2.44140683208e-05;
			processed.range_min = 0.0;
			processed.range_max = 6.0;

			processed.ranges.resize(NUM_OF_SCAN_POINTS);
    		processed.intensities.resize(NUM_OF_SCAN_POINTS);

			for (unsigned int i = 0; i < listOfSegments.size(); i++){
				Segment seg = listOfSegments[i];
				for (unsigned int j = 0; j < seg.pts.size(); j++){

					Point point = seg.pts[j];

					processed.ranges[point.num] = point.rho;
					processed.intensities[point.num] = i;
				}
			}


			//ROS_INFO("%u", listOfSegments.size());


			nav.data |= STOP;
			navPub.publish(nav);

			processPub.publish(processed);

			pts.clear();
			dataReady = false;
		}


		ros::spinOnce();
	}
}
