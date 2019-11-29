#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <cmath>
#include <string>
#include <priority_point.h>
#include <cost_map_ros/cost_map_ros.hpp>

// double makePath_AStar(std::vector<cost_map::Index> &path,PriorityPoint start,PriorityPoint goal,std::string layer,cost_map::CostMap &cost_map, double grid_cost, bool only_cost = false);
double makePath_ThetaStar(std::vector<cost_map::Index> &path,PriorityPoint start,PriorityPoint goal,std::string layer,cost_map::CostMap &cost_map, double grid_cost, bool only_cost = false);
double makePath_develop(std::vector<cost_map::Index> &path,PriorityPoint start,PriorityPoint goal,std::string layer,cost_map::CostMap &cost_map, double grid_cost, bool only_cost = false);
bool lineOfSight(int i1, int j1, int i2, int j2, cost_map::CostMap &cost_map, std::string layer, int minimal_cost);
float HeuristicEvclid(const PriorityPoint &source, const PriorityPoint &target,const double &unitCost);
unsigned int HeuristicChebishev(const PriorityPoint &source, const PriorityPoint &target,const double &unitCost);
unsigned int HeuristicManhetten(const PriorityPoint &source, const PriorityPoint &target,const double &unitCost);
unsigned int HeuristicManhetten(int* source, const PriorityPoint &target, const double &unit_cost);
double makePath_BoostThetaStar(std::vector<cost_map::Index> &path, PriorityPoint start, PriorityPoint goal, std::string layer, cost_map::CostMap &cost_map, double grid_cost, bool only_cost= false);

float HeuristicEvclid(int* source, const PriorityPoint &target, const double &unit_cost);
bool isNear(PriorityPoint &p1,PriorityPoint &p2,unsigned int dist);

#endif