#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_regulator.h>

int main(int argc, char **argv)
{
  /****************************************
  ** Ros
  ****************************************/
  ros::init(argc, argv, "trajectory_regulator");
  ros::NodeHandle node;
  float rate = 50;
  ros::Rate loop_rate(rate);

  /****************************************
  ** Env
  ****************************************/
  float speed = 0.5;
  nav_msgs::Odometry odom;
  TrajectoryRegulator trajectory_regulator = TrajectoryRegulator(rate, speed);
  /****************************************
  ** Pub
  ****************************************/
  trajectory_regulator.move_debug_pub = node.advertise<geometry_msgs::PoseStamped>("planner/tr/speed_debug", 1);
  trajectory_regulator.move_pub = node.advertise<geometry_msgs::Pose2D>("planner/tr/speed", 1);
  trajectory_regulator.is_stop_pub = node.advertise<std_msgs::Bool>("planner/tr/is_stop", 1);
  // trajectory_regulator.x_velocity_pub = node.advertise<geometry_msgs::Pose2D>("planner/tr/x_velocity", 1);
  // trajectory_regulator.x_velocity_pub = node.advertise<geometry_msgs::Pose2D>("planner/tr/y_velocity", 1);
  /****************************************
  ** Sub
  ****************************************/
  // ros::Subscriber sub1 = node.subscribe("/planner/lp/path", 2, &TrajectoryRegulator::callbackPathSimple, &trajectory_regulator);
  ros::Subscriber sub1 = node.subscribe("/planner/gp/path_simple", 2, &TrajectoryRegulator::callbackPathSimple, &trajectory_regulator);
  ros::Subscriber sub2 = node.subscribe("/real_corr", 2, &TrajectoryRegulator::callbackRealCorr, &trajectory_regulator);
  ros::Subscriber sub3 = node.subscribe("/cost_map_server/cost_map", 2, &TrajectoryRegulator::callbackCostMap, &trajectory_regulator);
  /****************************************
  ** Srv
  ****************************************/

  ros::ServiceServer service1 = node.advertiseService("planner/tr/Enable", &TrajectoryRegulator::callbackSrvEnable, &trajectory_regulator);
  ros::ServiceServer service2 = node.advertiseService("planner/tr/Mode", &TrajectoryRegulator::callbackSrvMode, &trajectory_regulator);

  /****************************************
  ** Wait for init
  ****************************************/
  while (!trajectory_regulator.is_init())
  {
//    ROS_INFO("wait for init");
    ros::spinOnce();
    loop_rate.sleep();
  }
  /****************************************
  ** Main loop
  ****************************************/
  while (node.ok())
  {
    trajectory_regulator.move();
    // ROS_INFO("move distance = %f",trajectory_regulator.moveSimple());
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
