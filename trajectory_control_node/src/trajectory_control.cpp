#include "trajectory_control.h"

TrajectoryControl::TrajectoryControl(int argc, char**argv)
{
    // Init atributes
    visualservo = false;
    current_waypoint = 0;
    margin = 1;
    landing = false;

    // Init desired position
    desired_pose.header.seq = 0;
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.header.frame_id = "map";

    desired_pose.pose.position.x = 0.0;
    desired_pose.pose.position.y = 0.0;
    desired_pose.pose.position.z = 0.0;
    
    desired_pose.pose.orientation.x = 0.0;
    desired_pose.pose.orientation.y = 0.0;
    desired_pose.pose.orientation.z = 1.0;
    desired_pose.pose.orientation.w = 0.0;
    
    // Pose subscriber to obtain CURRENT_POSE
    current_pose = node.subscribe("/uav1/mavros/local_position/pose", 1000, &TrajectoryControl::poseCheckCallback, this);
    
    // Aruco subscriber
    sub_aruco = node.subscribe("/uav1/aruco_single/pose",1000, &TrajectoryControl::arucoCallback, this);

    // Waypoint publisher
    pub_waypoints = node.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 1000);
    
    // Transform listener
    tfListener = new tf::TransformListener;

    // Landing service
    land_client = node.serviceClient<mavros_msgs::CommandTOL>("/uav1/mavros/cmd/land");

    // Define a list of Waypoints (3 or 4 different waypoints, your trajectory)
    // List of waypoints 
    geometry_msgs::Pose pose;
    
    pose.position.x = -5.0;
    pose.position.y = 0.2;
    pose.position.z = 10.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 1.0;
    pose.orientation.w = 0.0;
    trajectory_waypoints.push_back(pose);

    pose.position.x = -15.0;
    pose.position.y = 0.5;
    pose.position.z = 9;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 1.0;
    pose.orientation.w = 0.0;
    trajectory_waypoints.push_back(pose);

    pose.position.x = -25;
    pose.position.y = 1;
    pose.position.z = 8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 1.0;
    pose.orientation.w = 0.0;
    trajectory_waypoints.push_back(pose);
    
    pose.position.x = -32;
    pose.position.y = 1;
    pose.position.z = 6.7;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 1.0;
    pose.orientation.w = 0.0;
    trajectory_waypoints.push_back(pose);


    // WAYPOINTS navigation mode
    waypoints.header.seq = 0;
    waypoints.header.stamp = ros::Time::now();
    waypoints.header.frame_id = "map";
   
    waypoints.pose = trajectory_waypoints[current_waypoint];
    ROS_INFO("WAYPOINT navigation mode");

};

void TrajectoryControl::poseCheckCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Waypoint navigation until drone captures the aruco marker 
    if (equalPose(waypoints.pose, msg->pose) && !visualservo && !landing) {
        if (current_waypoint < 3) {
          current_waypoint++;
          ROS_INFO("WAYPOINT: %d", current_waypoint);
          waypoints.pose = trajectory_waypoints[current_waypoint];
        } else {
            visualservo = true;
            ROS_INFO("VISUAL SERVOING navigation mode");
        }
    }
    // Landing
    if(equalPose(desired_pose.pose, msg->pose) && visualservo && !landing) {
        landing = true;
        visualservo = false;
        mavros_msgs::CommandTOL land_cmd;
        land_cmd.request.altitude = 0;
        land_client.call(land_cmd);
        ROS_INFO("LANDING navigation mode");
    }
};

void TrajectoryControl::arucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!visualservo || landing) {
        return;
    }
    // Get the aruco transform (tf::Transform) cam_T_aruco
    tf::Transform cam_T_aruco;
    cam_T_aruco.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    cam_T_aruco.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
    
    // Get the desired offset transform (tf::Transform) desiredCam_T_aruco
    tf::Transform desiredCam_T_aruco;
    desiredCam_T_aruco.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z + margin));
    cam_T_aruco.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
 
    // Create a tf listener to obtain (tf::Transform) map_T_cam
    tf::StampedTransform map_STf_camera;
    tfListener->lookupTransform("map", "uav1/d435_front/color_optical_frame", ros::Time(0), map_STf_camera);
    tf::Transform map_Tf_cam(map_STf_camera.getBasis(), map_STf_camera.getOrigin());

    // Get bbaselink_ST_camera
    tf::StampedTransform baselink_STf_camera;
    tfListener->lookupTransform("uav1/base_link", "uav1/d435_front/color_optical_frame", ros::Time(0), baselink_STf_camera);
    tf::Transform baselink_ST_camera(baselink_STf_camera.getBasis(), baselink_STf_camera.getOrigin());

    // Compute the desired global waypoint (map_T_bl)
    tf::Transform map_T_bl = map_Tf_cam * cam_T_aruco * desiredCam_T_aruco.inverse() * baselink_ST_camera.inverse();

    // Publish the desired Waypoint you computed (from map_T_bl).
    waypoints.pose.position.x = map_T_bl.getOrigin().getX();
    waypoints.pose.position.y = map_T_bl.getOrigin().getY();
    waypoints.pose.position.z = map_T_bl.getOrigin().getZ();
    waypoints.pose.orientation.x = map_T_bl.getRotation().x();
    waypoints.pose.orientation.y = map_T_bl.getRotation().y();
    waypoints.pose.orientation.z = map_T_bl.getRotation().z();
    waypoints.pose.orientation.w = map_T_bl.getRotation().w();
    
    desired_pose = waypoints;  
};

void TrajectoryControl::publishWaypoint()
{
    pub_waypoints.publish(waypoints);
};

bool TrajectoryControl::equalPose(geometry_msgs::Pose waypointspose, geometry_msgs::Pose local_pose)
{
    float threshold = 2;

    // Compute the Euclidean norm of the pose error
    if(std::abs(waypointspose.position.x - local_pose.position.x) < threshold && 
    std::abs(waypointspose.position.y - local_pose.position.y) < threshold &&
    std::abs(waypointspose.position.z - local_pose.position.z) < threshold)
    {
        return true;
    }
    return false;
};
