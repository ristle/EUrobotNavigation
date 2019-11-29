#include <ros/ros.h>
#include <cost_map_ros/cost_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <path_planner.h>
#include <global_planner/Enable.h>
#include <global_planner/Mode.h>
#include <global_planner/Goal.h>
#include <global_planner/PathCost.h>

int main(int argc, char **argv)
{
  /****************************************
  ** Ros
  ****************************************/
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);

  //================== create input params env==============
  std::string path_layer;
  std::string path_force_layer;
  std::string frame_id;
  std::string current_position;
  std::string algorithm;
  double path_filter_epsilon;
  double euristic_grid_cost;
  //================== check all parametrs =================
  ROS_ASSERT(node.hasParam("global_planner/path_layer"));
  ROS_ASSERT(node.hasParam("global_planner/frame_id"));
  ROS_ASSERT(node.hasParam("global_planner/current_position"));
  ROS_ASSERT(node.hasParam("global_planner/path_filter_epsilon"));
  ROS_ASSERT(node.hasParam("global_planner/euristic_grid_cost"));
  ROS_ASSERT(node.hasParam("global_planner/algorithm"));
  //================== read input params ==============
  node.getParam("global_planner/path_layer", path_layer);
  node.getParam("global_planner/path_force_layer", path_force_layer);
  node.getParam("global_planner/frame_id", frame_id);
  node.getParam("global_planner/current_position", current_position);
  node.getParam("global_planner/path_filter_epsilon", path_filter_epsilon);
  node.getParam("global_planner/euristic_grid_cost", euristic_grid_cost);
  //===========================================================

  /****************************************
  ** Env
  ****************************************/
  geometry_msgs::Twist twist;
  PathPlanner path_planner(frame_id, path_layer, path_force_layer, path_filter_epsilon, euristic_grid_cost);
  /****************************************
  ** Pub
  ****************************************/
  path_planner.full_path_pub = node.advertise<nav_msgs::Path>("planner/gp/path_full", 2);
  path_planner.simple_path_pub = node.advertise<nav_msgs::Path>("planner/gp/path_simple", 2);
  path_planner.goal_now_pub = node.advertise<geometry_msgs::PoseStamped>("planner/gp/goal_now", 2);
  path_planner.goal_unreachable_pub = node.advertise<std_msgs::Bool>("planner/gp/goal_unreachable", 2);
  /****************************************
  ** Sub
  ****************************************/
  ros::Subscriber sub1 = node.subscribe("cost_map_server/cost_map", 1, &PathPlanner::callbackCostMap, &path_planner);
  while (!path_planner.isCostMapExist())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("got the map");

//  ros::Subscriber sub2 = node.subscribe("real", 1, &PathPlanner::callbackPoseOdomStart, &path_planner);
  ros::Subscriber sub2 = node.subscribe(current_position, 1, &PathPlanner::callbackPoseStampedCorrStart, &path_planner);  
  ros::Subscriber sub3 = node.subscribe("planner/goal", 1, &PathPlanner::callbackPoseStampedGoal, &path_planner);
  ros::Subscriber sub4 = node.subscribe("/clicked_point", 1, &PathPlanner::callbackPointStampedGoal, &path_planner);
  ros::Subscriber sub5 = node.subscribe("/initialpose", 1, &PathPlanner::callbackInitialpose, &path_planner);
  ros::Subscriber sub6 = node.subscribe("/move_base_simple/goal", 1, &PathPlanner::callbackMoveBaseSimpleGoal, &path_planner);
  /****************************************
  ** Srv
  ****************************************/
  ros::ServiceServer service1 = node.advertiseService("planner/Enable", &PathPlanner::callbackSrvEnable, &path_planner);
  ros::ServiceServer service2 = node.advertiseService("planner/Mode", &PathPlanner::callbackSrvMode, &path_planner);
  ros::ServiceServer service3 = node.advertiseService("planner/Goal", &PathPlanner::callbackSrvGoal, &path_planner);
  ros::ServiceServer service4 = node.advertiseService("planner/PathCost", &PathPlanner::callbackPathCost, &path_planner);
  path_planner.srv_enable_client = node.serviceClient<global_planner::Enable>("planner/tr/Enable");
  path_planner.srv_mode_client1 = node.serviceClient<global_planner::Mode>("planner/tr/Mode");
  path_planner.srv_mode_client2 = node.serviceClient<global_planner::Mode>("planner/lp/Mode");
  /****************************************
  ** Init loop
  ****************************************/

  while (!path_planner.is_init())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("start and goal points is recieved");
  /****************************************
  ** Main loop
  ****************************************/
  while (node.ok())
  {
    path_planner.update_path();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
