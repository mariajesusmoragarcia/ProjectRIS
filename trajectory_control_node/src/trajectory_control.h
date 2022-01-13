#ifndef CONTROL_TRAJECTORY_H
#define CONTROL_TRAJECTORY_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/CommandTOL.h>
#include <cmath>

class TrajectoryControl
{
    private:
      
      ros::NodeHandle node;									// Node handle
      
      ros::Publisher pub_waypoints;								// Publishers
      // ros::Publisher pub_visual;
    
      ros::Subscriber current_pose;								// Subscribers
      ros::Subscriber sub_aruco;
      
      tf::TransformListener* tfListener;							// Transform transform_listener

      ros::ServiceClient land_client;								// Service client
      
      												// Atributes
      bool visualservo;  									// VISUAL SERVOING navigation mode
      double margin; 	  									// Margin in front of the aruco

      geometry_msgs::PoseStamped waypoints;  							// Waypoints navigation mode
      std::vector<geometry_msgs::Pose> trajectory_waypoints; 				// Aruco waypoints trajectory 
      int current_waypoint; 									// Current waypoints trayectory 

      bool landing;										// LANDING navigation mode						
      geometry_msgs::PoseStamped desired_pose; 						// Desired position to landing. Final position

      bool equalPose(geometry_msgs::Pose waypoint, geometry_msgs::Pose local_pose);		// Methods
   
    public:
      												
      void poseCheckCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);		// Callbacks
      void arucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      
      void publishWaypoint();									// Methods

      TrajectoryControl(int argc, char**argv);
};

#endif
