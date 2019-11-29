#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelStamped.h>
#include <mover.h>

int main(int argc, char **argv)
{
  /****************************************
  ** Ros
  ****************************************/
  ros::init(argc, argv, "mover");
  ros::NodeHandle node;
  float loop_rate_hz = 10.0;
  ros::Rate loop_rate(loop_rate_hz);

  /****************************************
  ** Env
  ****************************************/
  float speed = 1.0;
  nav_msgs::Odometry odom;
  Mover mover = Mover(loop_rate_hz,speed);
  /****************************************
  ** Pub
  ****************************************/
  mover.accel_pub = node.advertise<geometry_msgs::AccelStamped>("mover/accel",1);
  mover.odom_pub = node.advertise<nav_msgs::Odometry>("odom",1);
  mover.real_pub = node.advertise<nav_msgs::Odometry>("real_corr",1);
  /****************************************
  ** Sub
  ****************************************/
  ros::Subscriber sub1 = node.subscribe("planner/gp/path_full",1,&Mover::callbackPathFull,&mover);
  ros::Subscriber sub2 = node.subscribe("planner/gp/path_simple",1,&Mover::callbackPathFull,&mover);
  ros::Subscriber sub3 = node.subscribe("initialpose",1,&Mover::callbackInitialPose, &mover);
  ros::Subscriber sub4 = node.subscribe("/planner/tr/speed",1,&Mover::callbackSpeed,&mover);
  /****************************************
  ** Srv
  ****************************************/
  // path_planner.path_search_client = node.serviceClient<path_planning::PathSearch>("PathSearch");
  /****************************************
  ** Main loop
  ****************************************/
  while (node.ok())
  {
    // mover.moveFull();
    mover.moveSpeed();
    // ROS_INFO("move distance = %f",mover.moveSimple());
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
