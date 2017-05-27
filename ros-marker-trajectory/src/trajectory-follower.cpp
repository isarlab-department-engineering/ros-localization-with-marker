/*
 * Copyright (C) 2017, Federico Parroni, Inc.
 *
 */

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16MultiArray.h" 

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/TransformStamped.h>

#include "aruco_detection/ArMarkers.h"

#include <math.h>

using namespace std;
using namespace geometry_msgs;


// ros publishers
ros::Publisher pub;

struct Marker {
    int id;
    Point position;
    double rotation;
};



// Feedback controller
float targetXDistance;
float targetZDistance = 0;
float targetAngle = 0;

const float Kx = 255/0.03;
const float Kz = 255/0.15;

// Filter
float estimatedLeftMPower;
float estimatedRightMPower;




//
// Find nearest marker index
// pos: 	translations vector
//
int indexOfNearest(const std::vector<double> pos) {
	int nearestIndex = 0;
	//double dist = sqrt(pow(pos[0],2.0) + pow(pos[1],2.0) + pow(pos[2],2.0));
	double dist = pos[2];

	for (int i = 3; i < pos.size(); i+=3)
	{
		//double dist2 = sqrt(pow(pos[i],2.0) + pow(pos[i+1],2.0) + pow(pos[i+2],2.0));
		double dist2 = pos[i+2];
		if(dist2 < dist) {
			dist = dist2;
			nearestIndex = i;
		}
	}
	return nearestIndex;
}


//
// Limit a value between two bounds
// value:		value to limit
// upperLimit:	upper bound
// lowerLimit:	lower bound
//
double limit(const double value, const double lowerLimit, const double upperLimit) {
	return value <= lowerLimit ? lowerLimit : value >= upperLimit ? upperLimit : value;
}

//
//	Receive msgs from 'markers_stream' topic
//
void markerCallback(aruco_detection::ArMarkers msg) {
	
  	if(msg.markerNo > 0) {
	  	// get nearest marker
	  	int indexRef = indexOfNearest(msg.tVecs);
	  	Marker marker;
	  	marker.id = msg.markersIds[indexRef];
	  	marker.position.x = msg.tVecs[indexRef];
	  	marker.position.y = msg.tVecs[indexRef+1];
	  	marker.position.z = msg.tVecs[indexRef+2];
	  	marker.rotation = msg.rVecs[indexRef+2];

	  	// errors
		double errorX = marker.position.x - targetXDistance;
		double errorZ = marker.position.z - targetZDistance;
		double errorAngle = marker.rotation - targetAngle;

		// z target
		short int leftMPower = limit(Kz * errorZ, 0.0, 255.0);
		short int rightMPower = limit(Kz * errorZ, 0.0, 255.0);

		ROS_INFO("%i / %i", leftMPower, rightMPower);
		// x target
		leftMPower = limit(leftMPower - Kx * errorX, 0.0, 255.0);
		rightMPower = limit(rightMPower + Kz * errorZ, 0.0, 255.0);

		ROS_INFO("%i / %i", leftMPower, rightMPower);
		// angle target



	  	std_msgs::Int16MultiArray motorSpeed;
	  	motorSpeed.data = {leftMPower,rightMPower,0,0};
	  	pub.publish(motorSpeed);

	} else {
		// stops motors if no marker found
		std_msgs::Int16MultiArray motorSpeed;
		motorSpeed.data = {0,0,0,0};
	  	pub.publish(motorSpeed);
	}
  
}


/*
 *
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_follower");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);

  n.getParam("xdistance", targetXDistance);

  // SUBSCRIBER
  ros::Subscriber sub = n.subscribe("/markers_stream", 5, markerCallback);

  // PUBLISHER   
  pub = n.advertise<std_msgs::Int16MultiArray>("/cmd", 5);

  while (ros::ok()) {     
	ros::spinOnce();
  }
  return 0;
}