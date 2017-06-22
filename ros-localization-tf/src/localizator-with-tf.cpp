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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "aruco_detection/ArMarkers.h"

#include <math.h>

using namespace std;
using namespace geometry_msgs;

// ros publishers
ros::Publisher pub;
ros::Publisher marker_pub;

struct Marker {
    int id;
    Point position;
    double rotation;
};

// TF and markers messages
bool publishRvizMarkers = false;

void sendBaseTF();
void sendMarkersTF();
visualization_msgs::Marker createMarkerMessage(const Marker marker, const std::string markerFrame);
void sendCurrentPositionTF(const Point msg, const float rotation);

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

		  	if(publishRvizMarkers) sendCurrentPositionTF(mapPos.point, marker.rotation - r.z);
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

  n.getParam("publish_rviz_markers", publishRvizMarkers);
  ROS_INFO("Show markers: %i", publishRvizMarkers);

  // SUBSCRIBER
  ros::Subscriber sub = n.subscribe("/markers_stream", 0, markerCallback);

  // PUBLISHER
  pub = n.advertise<geometry_msgs::PointStamped>("aruco_position", 0);
  if(publishRvizMarkers) marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker_visualization", 0);

  sendBaseTF();

  while (ros::ok())
  {
    ros::spinOnce();

    if(publishRvizMarkers) sendMarkersTF();

    loop_rate.sleep();
  }


  return 0;
}




// --- RVIZ ---

//
// create BASE TF -> (x,-z,y)
//
void sendBaseTF() {
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = "markersWorld";
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0.5*M_PI, 0, M_PI);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(static_transformStamped);
}


//
// Sends static markers tf
//
void sendMarkersTF() {
  static tf::TransformBroadcaster tfBroadcaster;

  // send markers tf
  visualization_msgs::MarkerArray markerArray;
  for(int i=0; i < markers.size(); ++i) {
	tf::Transform transform;
	Point p = markers[i].position;
	transform.setOrigin(tf::Vector3(p.x, p.y, p.z));

	tf::Quaternion q;
	q.setRPY(0, -markers[i].rotation, 0);
	transform.setRotation(q);
	
	std::string markerTFName = "marker#" + std::to_string(markers[i].id);
	tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "markersWorld", markerTFName));
	
	markerArray.markers.push_back(createMarkerMessage(markers[i], markerTFName));
  }

  marker_pub.publish(markerArray);
}

//
// Create visualization_msgs::Marker message
//
visualization_msgs::Marker createMarkerMessage(const Marker marker, const std::string markerFrame) {
	visualization_msgs::Marker m;

    m.header.frame_id = markerFrame;
    m.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
	m.ns = "localization";
    m.id = marker.id;
   
	m.type = visualization_msgs::Marker::CUBE;		// set the marker type
	m.action = visualization_msgs::Marker::ADD;		// Set the marker action. {ADD, DELETE, DELETEALL (new in Indigo 3)}
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	// Set the orientation -- quaternion
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;
	// Set the scale -- 1x1x1 means 1m per side
	m.scale.x = 0.08;
	m.scale.y = 0.08;
	m.scale.z = 0.003;
	// Set the color -- be sure to set alpha to something non-zero!
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.b = 1.0;
	m.color.a = 1.0;

	m.lifetime = ros::Duration();

	return m;
}


void sendCurrentPositionTF(const Point msg, const float rotation) {
  static tf::TransformBroadcaster tfBroadcaster;

  // send current position tf
  tf::Transform robotTf;
  robotTf.setOrigin( tf::Vector3(msg.x, msg.y, msg.z) );
  
  tf::Quaternion robotQ;
  robotQ.setRPY(0, rotation, 0);
  robotTf.setRotation(robotQ);

  tfBroadcaster.sendTransform(tf::StampedTransform(robotTf, ros::Time::now(), "markersWorld", "rospibot"));
}