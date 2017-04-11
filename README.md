Localization
=================

Estimate current position of the camera relative to a marker found by aruco_detector.
Subscribes to markers_stream by aruco_detection.
Publishes to aruco_position (geometry_msgs::PointStamped)

# Dependencies

Subscribes to aruco_detection topic:
- aruco_detection: [https://gitlab.com/duckietown/basicCV.git](https://gitlab.com/duckietown/basicCV)


# Description

Takes translation and rotation vector of the first found marker, calculates cartesian coordinates in the camera 3D system and retrieve cartesian coordinates of the camera relative to that marker by applying two coordinates transforms (translation and rotation).
Lots of maths!  

Marker map can be edited in YAML configuration file:

    <rosparam>
       markers: [
          { id: 11, position: [0.0,0.0,0.0], rotation: 0.0 },
          { id: 9,  position: [-0.40,0.0,0.20], rotation: 270.0 },
          { id: 10,  position: [-0.25,0.0,0.40], rotation: 180.0 }
       ]
    </rosparam>

It's also possible to send marker messages for rviz visualization:

    <param name="publish_rviz_markers" type="bool" value="true" />

# TODO

What to do when no marker found?
