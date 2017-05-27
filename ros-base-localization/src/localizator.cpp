/*
 * Copyright (C) 2017, Federico Parroni, Inc.
 *
 */

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

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

// markers in the map loaded from config file
std::vector<Marker> markers;


//
// Get new point coordinates applying a translation and a rotation to the initial axis
// point: 			input point to transform
// translation: 	new origin relative position
// rotation:		angle between old and new x-axis
//
Point getCoordSystemTransform(Point point, Point translation, float rotation) {
	Point p;
	float rot = roundf(rotation * 1000) / 1000;
	// ROS_INFO("Rot: %f gradi", rot * 180/3.14);

	float cosR = cos(rot);
	float sinR = sin(rot);
	p.x = (point.x - translation.x) * cosR + (point.z - translation.z) * sinR;
  	p.y = point.y - translation.y;
  	p.z = (point.z - translation.z) * cosR - (point.x - translation.x) * sinR;

  	return p;
}

//
// Get old point coordinates applying a translation and a rotation to the new axis
// point: 			input point to transform
// translation: 	old origin relative position
// rotation:		angle between old and new x-axis
//
Point getInverseCoordSystemTransform(Point point, Point translation, float rotation) {
	Point p;
	float rot = roundf(rotation * 1000) / 1000;
	// ROS_INFO("Rot: %f gradi", rot * 180/3.14);

	float cosR = cos(rot);
	float sinR = sin(rot);
	p.x = (point.x * cosR) - (point.z * sinR) + translation.x;
  	p.y = point.y + translation.y;
  	p.z = (point.x * sinR) + (point.z * cosR) + translation.z;

  	return p;
}

//
// Find in markers map if the visibile one exists
// id: 		marker detected in frame
// m:		marker to retrieve
//
bool findMarkerById(int id, Marker* m) {
	for(int i=0; i < markers.size(); ++i) {
		if(markers[i].id == id) {
			*m = markers[i];
			return true;
		}
	}
	return false;
}

//
//	Receive msgs from 'markers_stream' topic
//
void markerCallback(aruco_detection::ArMarkers msg) {
	Point r,t;
	int markerNo = msg.markerNo;
  	
  	if(markerNo > 0) {
	  	// analyze only first marker
	  	int id = msg.markersIds[0];

	  	// get marker transform
	  	Marker marker;
	  	if(findMarkerById(id, &marker)) {
		  	r.x = msg.rVecs[0];
		  	r.y = msg.rVecs[1];
		  	r.z = msg.rVecs[2];

		  	t.x = msg.tVecs[0];
		  	t.y = msg.tVecs[1];
		  	t.z = msg.tVecs[2];

		  	Point origin;
		  	origin.x = 0; origin.y = 0; origin.z = 0;

		  	// MARKER COORD SYSTEM
		  	float rotation = ( r.z > 0 ? M_PI - r.z : r.z);
		  	ROS_INFO("%f", r.z);

		  	Point mkrPos;
		  	mkrPos = getCoordSystemTransform(origin, t, rotation);
		  	mkrPos.z = abs(mkrPos.z);		// if angle > 0, z should be inversed, we take absolute value for semplicty

		  	// MAP COORD SYSTEM
		  	PointStamped mapPos;
		  	mapPos.point = getInverseCoordSystemTransform(mkrPos, marker.position, marker.rotation);
		  	mapPos.header.frame_id = "markersWorld";

		  	// for tracking purpose, msg.header.stamp is the time of currrent message
		  	//ROS_INFO("Time: %i", msg.header.stamp.nsec);

		  	ROS_INFO("I'm (x=%f, y=%f, z=%f) in MARKER %i coords system", mkrPos.x, mkrPos.y, mkrPos.z, id);
		  	ROS_INFO("I'm (x=%f, y=%f, z=%f) in MAP coords system", mapPos.point.x, mapPos.point.y, mapPos.point.z);

		  	pub.publish(mapPos);
	  	}

	} else {} // logic for search a marker
  
}

//
// Load markers configuration from YAML configuration file
//
//
void loadMarkersMap(const ros::NodeHandle n) {

	//ros::NodeHandle _nh("~");	// access private params
	XmlRpc::XmlRpcValue m;

    if(!n.getParam("markers", m))
        ROS_ERROR("Failed to read param 'markers'");
    else {
    	ROS_INFO("Markers found: %i", m.size());
    }

    for (int i=0; i<m.size(); ++i) {
        ROS_INFO("Id: %i, rotation: %f", (int)m[i]["id"], (double)m[i]["rotation"]);

        Marker curM;
        curM.id = (int)m[i]["id"];
        curM.position.x = (double)m[i]["position"][0];
        curM.position.y = (double)m[i]["position"][1];
        curM.position.z = (double)m[i]["position"][2];
        curM.rotation = (double)m[i]["rotation"] * (M_PI/180);

        markers.push_back(curM);
    }
}


/*
 *
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizator");

  ros::NodeHandle n("~");

  ros::Rate loop_rate(10);

  // load markers map
  loadMarkersMap(n);

  // SUBSCRIBER
  ros::Subscriber sub = n.subscribe("/markers_stream", 10, markerCallback);

  // PUBLISHER
  pub = n.advertise<geometry_msgs::PointStamped>("aruco_position", 10);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}