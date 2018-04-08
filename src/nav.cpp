#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/console.h>
#include "point.h"

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Byte.h"

using namespace std;

// number of range data points = 683

#define NUM_OF_SCAN_POINTS 683
#define MIN_RHO 0.07
#define SEPARATION 0.1
#define CORNER_ANGLE 1.0
#define MIN_PTS 10
#define BOTTOM_BOUND -0.2
#define UPPER_BOUND 3.2
#define FLOORLINE_BOTTOM_BOUND 1.0
#define FLOORLINE_UPPER_BOUND 2.0
#define SIDE_BOUND_FROM_CENTER 2.5
#define MIN_LENGTH 0.4
#define MIN_OBJECT_DISTANCE 0.2
#define OBSTACLE_DISTANCE 1.0
#define STOP 7


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

vector< vector<Point> > filterRaw(const vector<Point>& pts){
	bool last_corner;

	vector<Point> temp;
	vector< vector<Point> > groups;
	vector< vector<Point> > filteredGroups;

	for (int i = 0; i < pts.size(); i++){

		if (i > 7) {
			Point pt0 = pts[i],
				  pt1 = pts[i-1],
				  pt2 = pts[i-2],
				  pt3 = pts[i-3],
				  pt4 = pts[i-4],
				  pt5 = pts[i-5],
				  pt6 = pts[i-6],
				  pt7 = pts[i-7];

			float a = pt0 - pt1;
			float b = pt1 - pt2;

			float c = pt0 / pt1;
			float d = pt6 / pt7;

			bool close = a <= SEPARATION & b <= SEPARATION;
			bool corner = (abs(c - d) > CORNER_ANGLE) & !last_corner; 

			if (!close | corner) {
				groups.push_back(temp);
				temp.clear();
			}
			last_corner = corner;
		}
		temp.push_back(pts[i]);
	}

	groups.push_back(temp);
	temp.clear();

	for (unsigned int i = 0; i < groups.size(); i++){
		if (groups[i].size() > MIN_PTS) filteredGroups.push_back(groups[i]);
	}

	return filteredGroups;
}


vector<float> lineFit(const vector<Point>& segment){
	vector<float> coeff;

	unsigned int n = segment.size();
	float s_x = 0, s_y = 0, s_xx = 0, s_xy = 0, m = 0, avg_x = 0, avg_y = 0;

	for (unsigned int i = 0; i < n; i++){
		Point pt = segment[i];
		s_x += pt.x();
		s_y += pt.y();
		s_xx += pt.x()*pt.x();
		s_xy += pt.x()*pt.y();
	}	

	avg_x = s_x / n;
	avg_y = s_y / n;
	coeff.push_back(avg_x);
	coeff.push_back(avg_y);

	bool inBoundary = avg_x > BOTTOM_BOUND &
					  avg_x < UPPER_BOUND &
					  avg_y > -SIDE_BOUND_FROM_CENTER &
					  avg_y < SIDE_BOUND_FROM_CENTER;

	if (inBoundary){
		m = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
		coeff.push_back(m);
		coeff.push_back(avg_y - m * avg_x);

		Point last = segment.back();
		Point first = segment.front();
		coeff.push_back(last - first > MIN_LENGTH);
	}
    return coeff;
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

			int smallObjectDetected = 0, largeObjectDetected = 0;
			bool floorlineInRange = false;

			//ROS_INFO("%u",pts.size());
			vector<vector<Point> > groupsOfPts = filterRaw(pts);
			vector<vector<float> > lineFitCoeffs;
			vector<vector<float> > lines;
			vector<vector<float> > possibleObjects;
			vector<Point> objects;



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

			for (unsigned int i = 0; i < groupsOfPts.size(); i++){
				for (unsigned int j = 0; j < groupsOfPts[i].size(); j++){
					Point point = groupsOfPts[i][j];

					processed.ranges[point.num] = point.rho;
					processed.intensities[point.num] = i;
				}
				lineFitCoeffs.push_back(lineFit(groupsOfPts[i]));				
			}

			for (unsigned int i = 0; i < lineFitCoeffs.size(); i++){
				if (lineFitCoeffs[i].size() == 5){
					if(lineFitCoeffs[i][4]){
						lines.push_back(lineFitCoeffs[i]);
						ROS_INFO("%f %f", lineFitCoeffs[i][2], lineFitCoeffs[i][3]);
					} else possibleObjects.push_back(lineFitCoeffs[i]);
				}
			}

			if (!lines.empty()){
				for(unsigned int i = 0; i < lines.size(); i++){

					float m = lines[i][2];
					float c = lines[i][3];

					float floorIntercept = -c / m;

					if (floorIntercept < FLOORLINE_UPPER_BOUND & floorIntercept > FLOORLINE_BOTTOM_BOUND){
						floorlineInRange = true;
						
					}

					float deter = OBSTACLE_DISTANCE*OBSTACLE_DISTANCE*(m*m + 1.0) - c*c;

					if (deter > 0){
						float y1 = (c + abs(m)*sqrt(deter))/(m*m + 1.0);
						float y2 = (c - abs(m)*sqrt(deter))/(m*m + 1.0);

						bool a = (y1 < OBSTACLE_DISTANCE & y1 > OBSTACLE_DISTANCE/2);
						bool b = (y2 > -OBSTACLE_DISTANCE & y2 < -OBSTACLE_DISTANCE/2);

						largeObjectDetected |= 

						(a | (y2 < OBSTACLE_DISTANCE & y2 > OBSTACLE_DISTANCE/2) ) << 2 |

						((y1 > -OBSTACLE_DISTANCE/2 & y1 < OBSTACLE_DISTANCE/2) | (y2 > -OBSTACLE_DISTANCE/2 & y2 < OBSTACLE_DISTANCE/2) | (a&b)) << 1 |

						((y1 > -OBSTACLE_DISTANCE & y1 < -OBSTACLE_DISTANCE/2) | b);
					}
				}

				if (!floorlineInRange) {
					nav.data = STOP;
					ROS_WARN("FLOORLINE NOT IN RANGE");
				}


				for (unsigned int i = 0; i < possibleObjects.size(); i++){
					float x = possibleObjects[i][0];
					float y = possibleObjects[i][1];
					bool separate = true;
					for (unsigned int j = 0; j < lines.size(); j++){
						float m = lines[j][2];
						float c = lines[j][3];

						//Point to line distance
						bool onALine = abs(m*x - y + c)/sqrt(m*m + 1.0) < MIN_OBJECT_DISTANCE;
						if (onALine) {
							separate = false;
							break;
						}
					}
					if (separate){
						ROS_INFO("object at %f %f", x, y);
						objects.push_back(Point(x, y));					
					} 
				}
			} else {
				ROS_WARN("LIDAR IS NOT FACING ON THE FLOOR");
				nav.data = STOP;
			}

			ROS_INFO("---");

			for (unsigned int i = 0; i < objects.size(); i++){
				smallObjectDetected |= 

				(objects[i].theta > M_PI/6 &
				 objects[i].theta < M_PI_2 &
				 objects[i].rho < OBSTACLE_DISTANCE) << 2 |

				(objects[i].theta > -M_PI/6 &
				 objects[i].theta < M_PI/6 &
				 objects[i].rho < OBSTACLE_DISTANCE) << 1 |

				(objects[i].theta > -M_PI_2 &
				 objects[i].theta < -M_PI/6 &
				 objects[i].rho < OBSTACLE_DISTANCE);
			}


			nav.data |= largeObjectDetected | smallObjectDetected;
			
			navPub.publish(nav);
			processPub.publish(processed);

			pts.clear();
			dataReady = false;
		}


		ros::spinOnce();
	}
}
