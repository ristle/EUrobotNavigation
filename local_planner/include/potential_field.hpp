#ifndef POTENTIAL_FIELD
#define POTENTIAL_FIELD

#include <ros/ros.h>

#include <algorithm>
#include <iostream>
#include <limits.h>
#include <iterator>
#include <iomanip>
#include <math.h>
#include <vector>

#include <cost_map_ros/cost_map_ros.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <cost_map_ros/converter.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <path_simplifier.cpp>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <iterator>
#include <stdio.h>
#include <math.h>

#define pair_double std::pair<double, double>

double CalcDistance(std::pair<double, double> start, std::pair<double, double> end) {
    return sqrt(pow(start.first - end.first, 2) + pow(start.second - end.second, 2));
}

pair_double transformPose(geometry_msgs::PoseStamped &pose) {
    return pair_double {pose.pose.position.x, pose.pose.position.y};
}

double compare(double x, double y) {
    if (x == y)
        return x - y;

    return x > y ? -1 : 1;
}

// Создание Attractive коэфф. для потенциального поля
double CalcAttractivePotential(grid_map::Index index, double goalx, double goaly) {
    return sqrt(pow(index[0] - goalx, 2) + pow(index[1] - goaly, 2));
}

// Создание потенциального поля
void CalcPotential(double startx, double starty, double goalx, double goaly, cost_map::CostMap &cost_map,
                   double (&our_map)[300][200]) {
    // Use some magic for normalisation of Field
    double max_distance = (CalcAttractivePotential(grid_map::Index(startx, starty), goalx, goaly) + 15);
    if (max_distance == 0.0) {
        max_distance = 0.01;
    }
    for (cost_map::CircleIterator iterator(cost_map, grid_map::Position(startx / 100.0, starty / 100.0),
                                           max_distance / 100.);
         !iterator.isPastEnd(); ++iterator) {
        try {
            grid_map::Index index(*iterator);
            double uf;
            uf = cost_map.at("obstacle_inflation_layer", *iterator);

            // if we on free podouble - field is more less then if it not free podouble
            if (uf >= 10) {
                uf = 3000.0; // CP - is const variable
            } else
                uf += CalcAttractivePotential(index, goalx, goaly) / max_distance * 256;

            our_map[299 - index(0)][199 - index(1)] = uf;
        }
        catch (std::exception &e) {
            ROS_INFO("Exception! %s", e.what());
        }
    }
}

// Создание пути
void get_new_pose(cost_map::CostMap &cost_map, nav_msgs::Path *path, nav_msgs::Path *output_path,
                  double ( &our_map)[300][200], double startx, double starty, double max_distance) {
    // We are here if  output_path.size > 1
    // Find Gradient of x and y
    size_t last = path->poses.size() - 1;
    double new_pose_x = startx;
    double new_pose_y = starty;
    double goalx = path->poses[last].pose.position.x;
    double goaly = path->poses[last].pose.position.y;
    double global_counter = 0;
    double variable = 0;
    double counter = 0;
    double index;

    bool build = false;
    output_path->poses.clear();

    size_t path_len = path->poses.size() - 1;

    CalcPotential(startx, starty,
                  path->poses[path_len].pose.position.x * 100, path->poses[path_len].pose.position.y * 100, cost_map,
                  our_map);
    ROS_INFO_THROTTLE(1, " CalcPotential Done");
    for (double i = 0; i < path->poses.size() - 1; i++) {
        geometry_msgs::PoseStamped p;
        p.pose.position.x = path->poses[i].pose.position.x * 100.;
        p.pose.position.y = path->poses[i].pose.position.y * 100;
        output_path->poses[i] = p;
    }

    // Смещение Градиентным спуском
    for (double i = 1; i < output_path->poses.size() - 1; i++) {
        double new_pose_x = output_path->poses[i].pose.position.x;
        double new_pose_y = output_path->poses[i].pose.position.y;
        counter = 0;

        double doubleensive = our_map[(int) new_pose_x][(int) new_pose_y] - CalcAttractivePotential(
                grid_map::Index(new_pose_x, new_pose_y), goalx, goaly) *
                                                                            256 / max_distance;
        ROS_INFO_THROTTLE(1, " Got doubleensive ");
        while (doubleensive >= 2200) {
            new_pose_x += compare(
                    our_map[(int) new_pose_x + 1][(int) new_pose_y],
                    our_map[(int) new_pose_x - 1][(int) new_pose_y]);

            new_pose_y += compare(
                    our_map[(int) new_pose_x][(int) new_pose_y + 1],
                    our_map[(int) new_pose_x][(int) new_pose_y - 1]);

            new_pose_x = new_pose_x > 299 ? 299 : new_pose_x;
            new_pose_y = new_pose_y > 199 ? 199 : new_pose_y;

            new_pose_x = new_pose_x < 1 ? 0 : new_pose_x;
            new_pose_y = new_pose_y < 1 ? 0 : new_pose_y;
            doubleensive = our_map[(int) new_pose_x][(int) new_pose_y] - CalcAttractivePotential(
                    grid_map::Index(new_pose_x, new_pose_y), goalx, goaly) *
                                                                         256 / max_distance;
            ROS_INFO_THROTTLE(1, " in While got doubleensive ");
            counter++;
            if (counter > 15) {
                break;
            }
        }

        output_path->poses[i].pose.position.x = new_pose_x;
        output_path->poses[i].pose.position.y = new_pose_y;
    }
}

#endif