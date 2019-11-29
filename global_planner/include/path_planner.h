#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#define MIN_COST_OBSTACLE 10

#include <ros/ros.h>
#include <cost_map_ros/converter.hpp>
#include <cost_map_ros/cost_map_ros.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Point.h>

#include <limits>
#include <cmath>
#include <string>
#include <utility>
#include <stdexcept>

#include <priority_point.h>
#include <path_simplifier.h>
#include <global_planner.h>
#include <global_planner/Enable.h>
#include <global_planner/Mode.h>
#include <global_planner/Goal.h>
#include <global_planner/PathCost.h>

class PathPlanner
{
public:
  ros::Publisher full_path_pub;
  ros::Publisher simple_path_pub;
  ros::Publisher twist_pub;
  ros::Publisher goal_unreachable_pub;
  ros::Publisher local_path_pub;
  ros::Publisher goal_now_pub;
  ros::ServiceClient srv_enable_client;
  ros::ServiceClient srv_mode_client1;
  ros::ServiceClient srv_mode_client2;

  PathPlanner(std::string frame_id_, std::string path_layer_, std::string path_force_layer_, double path_filter_epsilon_, double euristic_grid_cost_)
  {
    cost_map = cost_map::CostMap();
    frame_id = frame_id_;
    path_filter_epsilon = path_filter_epsilon_;
    path_layer = path_layer_;
    path_force_layer = path_force_layer_;
    euristic_grid_cost = euristic_grid_cost_;
    iStart = {0, 0};
    iGoal = {0, 0};
    pStart = {0, 0};
    pGoal = {0, 0};
  }
  bool isCostMapExist()
  {
    return cost_map.getFrameId() == frame_id;
  }
  bool is_init()
  {
    return (iStart[0] != 0 && iStart[1] != 0 && iGoal[0] != 0 && iGoal[0] != 0);
  }

  bool callbackSrvEnable(global_planner::Enable::Request &req, global_planner::Enable::Response &res)
  {
    enable_msg.request.enable.data = req.enable.data;
    ROS_INFO("got Enable move: %s", req.enable.data == true ? "true" : "false");
    srv_enable_client.call(enable_msg);
    return true;
  }
  bool callbackSrvMode(global_planner::Mode::Request &req, global_planner::Mode::Response &res)
  {
    obstacle = req.obstacle.data;
    mode_msg.request.obstacle.data = obstacle;
    mode_msg.request.speed.data = req.speed.data;
    mode_msg.request.acceleration.data = req.acceleration.data;
    mode_msg.request.braking.data = req.braking.data;
    ROS_INFO("got Mode\n obstacle: %s\n speed: %f\n acceleration: %f\n braking: %f\n", obstacle == true ? "On" : "Off", mode_msg.request.speed.data, mode_msg.request.acceleration.data, mode_msg.request.braking.data);
    srv_mode_client1.call(mode_msg);
    srv_mode_client2.call(mode_msg);
    return true;
  }
  bool callbackSrvGoal(global_planner::Goal::Request &req, global_planner::Goal::Response &res)
  {
    pGoal = {req.goal_point.pose.position.x, req.goal_point.pose.position.y};
    checkOrPutPointOnMap(pGoal, cost_map);
    cost_map.getIndex(pGoal, iGoal);
    angle_goal = req.goal_point.pose.orientation;
    ROS_INFO("got goal:\n x %f\n y %f\n", pGoal[0], pGoal[1]);
    return true;
  }
  bool callbackPathCost(global_planner::PathCost::Request &req, global_planner::PathCost::Response &res)
  {

    std::string search_layer = req.with_static.data? path_layer:path_force_layer;
    cost_map::Index iGoal, iStart;
    
    cost_map::Position pGoal = {req.goal_point.pose.position.x, req.goal_point.pose.position.y};
    cost_map::Position pStart = {req.start_point.pose.position.x, req.start_point.pose.position.y};

    checkOrPutPointOnMap(pGoal, cost_map);
    checkOrPutPointOnMap(pStart, cost_map);
    
    cost_map.getIndex(pGoal, iGoal);
    cost_map.getIndex(pStart, iStart);
    
    std_msgs::Bool msg_unreachable;
    msg_unreachable.data = (cost_map.at(search_layer, iGoal) > MIN_COST_OBSTACLE) ? true : false; // TODO: remove hardcode
    if (msg_unreachable.data)
      {   
        res.path_len.data = -1;
        return true;
      }
    msg_unreachable.data = (cost_map.at(search_layer, iStart) > MIN_COST_OBSTACLE) ? true : false; // TODO: remove hardcode
    if (msg_unreachable.data)
      {   
        res.path_len.data = -1;
        return true;
      }
    
    PriorityPoint start = {iStart[0], iStart[1]};
    PriorityPoint goal = {iGoal[0], iGoal[1]};
    
    res.path_len.data = makePath_ThetaStar(path_vector, start, goal, search_layer, cost_map, euristic_grid_cost, 1); ;

    return true;
  }

  void callbackCostMap(const cost_map_msgs::CostMap::ConstPtr &msg)
  {
    cost_map::fromMessage(*msg, cost_map);
  }
  void callbackPoseStampedCorrStart(const nav_msgs::Odometry &msg)
  {
    pStart = {msg.pose.pose.position.x, msg.pose.pose.position.y};
    checkOrPutPointOnMap(pStart, cost_map);
    cost_map.getIndex(pStart, iStart);
  }

  void callbackPoseStampedStart(const geometry_msgs::PoseStamped &msg)
  {
    pStart = {msg.pose.position.x, msg.pose.position.y};
    checkOrPutPointOnMap(pStart, cost_map);
    cost_map.getIndex(pStart, iStart);
  }
  void callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped &msg)
  {
    pStart = {msg.pose.pose.position.x, msg.pose.pose.position.y};
    checkOrPutPointOnMap(pStart, cost_map);
    cost_map.getIndex(pStart, iStart);
  }
  void callbackPoseOdomStart(const nav_msgs::Odometry &msg)
  {
    pStart = {msg.pose.pose.position.x, msg.pose.pose.position.y};
    checkOrPutPointOnMap(pStart, cost_map);
    cost_map.getIndex(pStart, iStart);
  }
  void callbackMoveBaseSimpleGoal(const geometry_msgs::PoseStamped &msg)
  {
    pGoal = {msg.pose.position.x, msg.pose.position.y};
    checkOrPutPointOnMap(pGoal, cost_map);
    cost_map.getIndex(pGoal, iGoal);
    angle_goal = msg.pose.orientation;
  }
  void callbackPoseStampedGoal(const geometry_msgs::PoseStamped &msg)
  {
    pGoal = {msg.pose.position.x, msg.pose.position.y};
    checkOrPutPointOnMap(pGoal, cost_map);
    cost_map.getIndex(pGoal, iGoal);
    angle_goal = msg.pose.orientation;
  }
  void callbackPointStampedGoal(const geometry_msgs::PointStamped &msg)
  {
    pGoal = {msg.point.x, msg.point.y};
    checkOrPutPointOnMap(pGoal, cost_map);
    cost_map.getIndex(pGoal, iGoal);
    geometry_msgs::Quaternion zero_angle;
    zero_angle.w = 1;
    zero_angle.x = 0;
    zero_angle.y = 0;
    zero_angle.z = 0;
    angle_goal = zero_angle;
  }
  void publish_goal_now(cost_map::Position pose,geometry_msgs::Quaternion& angle)
  {
    geometry_msgs::PoseStamped Goal;

    Goal.header.frame_id = "map";
    Goal.header.stamp = ros::Time::now();

    Goal.pose.position.x = pose[0];
    Goal.pose.position.y = pose[1];
    Goal.pose.orientation = angle;
    goal_now_pub.publish(Goal);
  }

  void printInfo()
  {
    std::string layersStr = "cost_map layers: ";
    for (unsigned int i = 0; i < cost_map.getLayers().size(); ++i)
    {
      layersStr += (cost_map.getLayers().at(i) + " ");
    }
    ROS_INFO("%s", layersStr.c_str());
    ROS_INFO("cost_map size: %ix%i", cost_map.getSize()[0], cost_map.getSize()[1]);
    ROS_INFO("cost_map resolution: %f m/cell", cost_map.getResolution());
    ROS_INFO("start position: %fx%f at index: %ix%i", pStart[0], pStart[1], iStart[0], iStart[1]);
    ROS_INFO("goal position: %fx%f at index: %ix%i", pGoal[0], pGoal[1], iGoal[0], iGoal[1]);
  }

  void update_path()
  {
    std::string search_layer = obstacle ? path_layer : path_force_layer;
    msg_unreachable.data = (cost_map.at(search_layer, iGoal) > MIN_COST_OBSTACLE) ? true : false; // TODO: remove hardcode
    // check that we already on goal point
    if(is_points_simular(pStart,pGoal,0.01))
    {
      clear_path();
      publish_state();
      return;
    } 

    PriorityPoint start = {iStart[0], iStart[1]};
    PriorityPoint goal = {iGoal[0], iGoal[1]};

    makePath_ThetaStar(path_vector, start, goal, search_layer, cost_map, euristic_grid_cost);
    create_ROS_path(full_path, path_vector, angle_goal, frame_id);
    FilterRosPath(simple_path, full_path, path_filter_epsilon);
    publish_state();
  }

private:
  cost_map::CostMap cost_map;
  cost_map::Index iStart, iGoal;
  cost_map::Position pStart, pGoal;
  nav_msgs::Path full_path, simple_path;
  geometry_msgs::Quaternion angle_goal;
  double path_filter_epsilon;
  std::string path_layer;
  std::string path_force_layer;
  std::string frame_id;
  bool obstacle = true;
  std::vector<cost_map::Index> path_vector;
  double euristic_grid_cost;
  global_planner::Enable enable_msg;
  global_planner::Mode mode_msg;
  std_msgs::Bool msg_unreachable;

  bool is_points_simular(const cost_map::Position &p1, const cost_map::Position &p2, float min_diff){
    return (fabs(p1[0] - p2[0]) < min_diff) && (fabs(p1[1] - p2[1]) < min_diff);
  }

  void publish_state(){
    cost_map::Position Goal = pGoal;
    // cost_map::Position Goal = {-1, -1};
    // Goal = msg_unreachable.data?pGoal:Goal;
    publish_goal_now(Goal,angle_goal);
    full_path_pub.publish(full_path);
    simple_path_pub.publish(simple_path);
    return;
  }

  bool checkOrPutPointOnMap(cost_map::Position &point, const cost_map::CostMap &map)
  {
    double lengtX = map.getLength().x();
    double lengtY = map.getLength().y();
    double resolution = map.getResolution();
    cost_map::Position temp = point;
    point[0] = point[0] < resolution ? resolution : point[0];
    point[1] = point[1] < resolution ? resolution : point[1];
    point[0] = point[0] > (lengtX - resolution) ? lengtX - resolution : point[0];
    point[1] = point[1] > (lengtY - resolution) ? lengtY - resolution : point[1];
    bool pointOnMap = (point == temp);
    if (!pointOnMap)
    {
      ROS_INFO("point is out of map");
    }
    return pointOnMap;
  }

  void clear_path()
  {
    full_path.poses.clear();
    full_path.header.stamp = ros::Time::now();
    full_path.header.frame_id = frame_id;
    full_path_pub.publish(simple_path);
    simple_path.poses.clear();
    simple_path.header.stamp = ros::Time::now();
    simple_path.header.frame_id = frame_id;
    simple_path_pub.publish(simple_path);
  }
  void create_ROS_path(nav_msgs::Path &path_msg, std::vector<cost_map::Index> &path, geometry_msgs::Quaternion &angle, std::string frame) //FIXME: please fix this sheet
  {
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame;
    path_msg.poses.clear();
    geometry_msgs::PoseStamped p;
    cost_map::Position pose;
    for (int it = 0; it < path.size(); it++)
    {
      cost_map.getPosition(path[it], pose);
      p.header.stamp = path_msg.header.stamp;
      p.header.frame_id = path_msg.header.frame_id;
      p.pose.position.x = pose.x();
      p.pose.position.y = pose.y();
      p.pose.position.z = 0;
      p.pose.orientation = angle;
      path_msg.poses.push_back(p);
    }
  }
  bool IsLayerExist(const std::string &layer, const cost_map::CostMap &map)
  {
    for (unsigned int i = 0; i < map.getLayers().size(); ++i)
    {
      if (map.getLayers().at(i) == layer)
      {
        return true;
      }
    }
    return false;
  }
};
#endif
