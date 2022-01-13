#include "trajectory_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_trajectory");
 
  TrajectoryControl trajectory(argc, argv);

  ros::Rate loop_rate(50); // Rate to publishing the waypoint
  while (ros::ok()) 
  {
    trajectory.publishWaypoint();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
