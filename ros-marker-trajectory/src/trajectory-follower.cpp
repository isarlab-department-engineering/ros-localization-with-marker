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
float targetXDistance = 0.1;
const float K = 255/1.4;
//const float Kx = 255/0.4;


// Filter
const float filterK = 0.75;
float estimatedLeftMPower;
float estimatedRightMPower;



//
// Point convenience initializer
//
Point CreatePoint(float x, float y, float z) {
	Point p;
	p.x = x; p.y = y; p.z = z;
	return p;
}

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
	return (value <= lowerLimit ? lowerLimit : (value >= upperLimit ? upperLimit : value));
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

		Point currentPos = CreatePoint(0, 0, 0);
		Point setpoint = CreatePoint(marker.position.x - targetXDistance, marker.position.y, marker.position.z - 0.20);
	  	// errors
		Point error = CreatePoint(setpoint.x - currentPos.x, setpoint.y - currentPos.y, setpoint.z - currentPos.z);
	
		ROS_INFO("(%f, %f)",error.x, error.z);
		//double errorAngle = marker.rotation - targetAngle;

		// pos target
		short int leftMPower = 80 +  (error.x > 0 ? limit(K * error.x, -120, 120) : 0);//limit(Kz * errorZ, 0.0, 255.0)/2;
		short int rightMPower = 80 + (error.x > 0 ? 0 : -limit(K * error.x, -120, 120));//limit(Kz * errorZ, 0.0, 255.0)/2;

		estimatedLeftMPower = (1-filterK)*estimatedLeftMPower  + filterK * leftMPower;
		estimatedRightMPower= (1-filterK)*estimatedRightMPower + filterK * rightMPower;

	  	std_msgs::Int16MultiArray motorSpeed;
	  	motorSpeed.data = {estimatedLeftMPower,estimatedRightMPower,0,0};
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
	loop_rate.sleep();
  }
  return 0;
}
