
#include <ros/ros.h>
#include <cstdlib>
#include <cost_map_ros/cost_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <cost_map_server.h>

int main(int argc, char **argv)
{
  /****************************************
  ** Ros
  ****************************************/
  ros::init(argc, argv, "cost_map_server");
  ros::NodeHandle node;
  int rate = 50;
  int timer = 20;
  ros::Rate loop_rate(rate);
  //================== create input params env==============
  double big_robot_size;
  double little_robot_size;
  double cube_size;
  double inscribed_radius;
  double inflation_radius;
  double inflation_exponential_rate;
  double min_diff_points;
  std::string big_robot1;
  std::string big_robot2;
  std::string small_robot1;
  std::string small_robot2;
  std::string side = "hz";
  //================== check all parametrs =================
  ROS_ASSERT(node.hasParam("cost_map_server/big_robot_size"));
  ROS_ASSERT(node.hasParam("cost_map_server/little_robot_size"));
  ROS_ASSERT(node.hasParam("cost_map_server/inscribed_radius"));
  ROS_ASSERT(node.hasParam("cost_map_server/inflation_radius"));
  ROS_ASSERT(node.hasParam("cost_map_server/inflation_exponential_rate"));
  ROS_ASSERT(node.hasParam("cost_map_server/big_robot1"));
  ROS_ASSERT(node.hasParam("cost_map_server/big_robot2"));
  ROS_ASSERT(node.hasParam("cost_map_server/small_robot1"));
  ROS_ASSERT(node.hasParam("cost_map_server/small_robot2"));
  ROS_ASSERT(node.hasParam("cost_map_server/min_diff_points"));
  //================== read input params ==============
  node.getParam("side", side);
  node.getParam("cost_map_server/big_robot_size", big_robot_size);
  node.getParam("cost_map_server/little_robot_size", little_robot_size);
  node.getParam("cost_map_server/inscribed_radius", inscribed_radius);
  node.getParam("cost_map_server/inflation_radius", inflation_radius);
  node.getParam("cost_map_server/inflation_exponential_rate", inflation_exponential_rate);
  node.getParam("cost_map_server/big_robot1", big_robot1);
  node.getParam("cost_map_server/big_robot2", big_robot2);
  node.getParam("cost_map_server/small_robot1", small_robot1);
  node.getParam("cost_map_server/small_robot2", small_robot2);
  node.getParam("cost_map_server/min_diff_points", min_diff_points);
  /****************************************
  ** Env
  ****************************************/
  std::string image_resource_name = argv[1]; //path to config xml from node start args[1]
  CostMapServer costmapserver(image_resource_name,
                              big_robot_size, little_robot_size,
                              inscribed_radius, inflation_radius,
                              inflation_exponential_rate, min_diff_points, side);
  /****************************************
  ** Pub.They publish on change sub
  ****************************************/
  costmapserver.static_publisher = node.advertise<nav_msgs::OccupancyGrid>("cost_map_server/static_layer", 1, true);
  costmapserver.inflation_publisher = node.advertise<nav_msgs::OccupancyGrid>("cost_map_server/inflation_layer", 1, true);
  costmapserver.inflation_half_publisher = node.advertise<nav_msgs::OccupancyGrid>("cost_map_server/inflation_half_layer", 1, true);
  costmapserver.paint_publisher = node.advertise<nav_msgs::OccupancyGrid>("cost_map_server/paint_layer", 1, true);
  costmapserver.obstacle_inflation_publisher = node.advertise<nav_msgs::OccupancyGrid>("cost_map_server/obstacle_inflation", 1, true);
  costmapserver.cost_map_publisher = node.advertise<cost_map_msgs::CostMap>("cost_map_server/cost_map", 1, true);
  /****************************************
  ** Sub
  ****************************************/
  ros::Subscriber sub1 = big_robot2 != "null" ? node.subscribe(big_robot2, 2, &CostMapServer::callbackBigEnemy, &costmapserver) : ros::Subscriber();
  ros::Subscriber sub3 = big_robot1 != "null" ? node.subscribe(big_robot1, 2, &CostMapServer::callbackMyBigFriend, &costmapserver) : ros::Subscriber();
  ros::Subscriber sub2 = small_robot2 != "null" ? node.subscribe(small_robot2, 2, &CostMapServer::callbackLittleEnemy, &costmapserver) : ros::Subscriber();
  ros::Subscriber sub4 = small_robot1 != "null" ? node.subscribe(small_robot1, 2, &CostMapServer::callbackMyLittleFriend, &costmapserver) : ros::Subscriber();

  costmapserver.PublishStaticLayer(); //ones if it latched
  costmapserver.PublishDynamicLayer();//for path_planner

  while (node.ok())
  {
    costmapserver.UpdateMap();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
