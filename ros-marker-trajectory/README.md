Trajectory Follower
=================

Follows a markers trajectory.
Subscribes to markers_stream by aruco_detection.
Publishes to motor_hat (geometry_msgs::PointStamped)

# Dependencies

Subscribes to aruco_detection topic:
- aruco_detection: [https://gitlab.com/duckietown/basicCV.git](https://gitlab.com/duckietown/basicCV)


# Parameters

Can be launched by typing:
    roslaunch ros_marker_trajectory aio_trajectory_follower.launch

Requires one param:
    <param name="xdistance" value="0.1" />
