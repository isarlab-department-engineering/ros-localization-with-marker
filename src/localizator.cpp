/*
 * Copyright (C) 2017, Federico Parroni, Inc.
 *
 */

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include "aruco_detection/ArMarkers.h"

#include <math.h>

using namespace std;
using namespace geometry_msgs;

// ros basic elements
ros::Publisher pub;
ros::Publisher marker_pub;

struct Marker {
    int id;
    Point position;
    double rotation;
};

bool publishRvizMarker = false;


void sendTF(Point msg, float rotation);
const string FRAME_NAME = "/marker_map";

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
  	
  	// LOCALIZE based on found marker position
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
		  	mapPos.header.frame_id = FRAME_NAME;

		  	// for tracking purpose, msg.header.stamp is the time of currrent message
		  	//ROS_INFO("Time: %i", msg.header.stamp.nsec);

		  	ROS_INFO("I'm (x=%f, y=%f, z=%f) in MARKER %i coords system", mkrPos.x, mkrPos.y, mkrPos.z, id);
		  	ROS_INFO("I'm (x=%f, y=%f, z=%f) in MAP coords system", mapPos.point.x, mapPos.point.y, mapPos.point.z);

		  	sendTF(mapPos.point, rotation);
		  	pub.publish(mapPos);
	  	}

	} else {} // logic for search a marker
  
}



//
// Load markers configuration from YAML configuration file
//
//
void loadMarkersMap(ros::NodeHandle n) {

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


//
// Send marker msg to rviz
//
//
void sendMarkerMessage(ros::NodeHandle n) {
	const uint32_t shape = visualization_msgs::Marker::CUBE;

	for(int i=0; i<markers.size(); i++) {
		visualization_msgs::Marker marker;

	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = FRAME_NAME;
	    marker.header.stamp = ros::Time::now();
	    // Set the namespace and id for this marker.  This serves to create a unique ID
	      // Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "localization";
	    marker.id = markers[i].id;
	   
		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = shape;

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = markers[i].position.x;
		marker.pose.position.y = markers[i].position.y;
		marker.pose.position.z = markers[i].position.z;
		
		tf::Quaternion quat;
  		quat.setRPY(0, 0, markers[i].rotation);
  		geometry_msgs::Quaternion geomQ = marker.pose.orientation;
  		tf::quaternionTFToMsg(quat, geomQ);
		marker.pose.orientation = geomQ;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.002;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		marker_pub.publish(marker);
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

  n.getParam("publish_rviz_markers", publishRvizMarker);
  ROS_INFO("Show markers: %i", publishRvizMarker);

  // SUBSCRIBER
  ros::Subscriber sub = n.subscribe("/markers_stream", 10, markerCallback);

  // PUBLISHER
  pub = n.advertise<geometry_msgs::PointStamped>("aruco_position", 50);
  if(publishRvizMarker) marker_pub = n.advertise<visualization_msgs::Marker>("marker_visualization", 1);


  while (ros::ok())
  {
    ros::spinOnce();

    if(publishRvizMarker) sendMarkerMessage(n);

    loop_rate.sleep();
  }


  return 0;
}






void sendTF(Point msg, float rotation) {
  static tf::TransformBroadcaster tfBroadcaster;
  // create opencv BASE TF * (x,-z,y)
  tf::Transform baseTf;
  baseTf.setOrigin( tf::Vector3(0,0,0) );
  tf::Quaternion baseQ;
  baseQ.setRPY(0.5*M_PI, 0, M_PI);
  baseTf.setRotation(baseQ);
  tfBroadcaster.sendTransform(tf::StampedTransform(baseTf, ros::Time::now(), "world", "myWorld"));

  // send markers tf
  for(int i=0; i < markers.size(); ++i) {
	tf::Transform transform;
	Point p = markers[i].position;
	transform.setOrigin(tf::Vector3(p.x, p.y, p.z));

	tf::Quaternion q;
	q.setRPY(0, -markers[i].rotation, 0);
	transform.setRotation(q);
	
	std::string tfName = "marker#" + std::to_string(markers[i].id);
	tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "myWorld", tfName));
  }

  // send current position tf
  tf::Transform robotTf;
  robotTf.setOrigin( tf::Vector3(msg.x, msg.y, msg.z) );
  
  tf::Quaternion robotQ;
  robotQ.setRPY(0, rotation, 0);
  robotTf.setRotation(robotQ);

  tfBroadcaster.sendTransform(tf::StampedTransform(robotTf, ros::Time::now(), "myWorld", "rospibot"));
}