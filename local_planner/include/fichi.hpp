#ifndef FICHI_HPP
#define FICHI_HPP

#include <ros/ros.h>

#include <algorithm>
#include <iostream>
#include <limits.h>
#include <iterator>
#include <iomanip>
#include <math.h>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include "potential_field.hpp"

#include <cost_map_ros/cost_map_ros.hpp>
#include <cost_map_ros/converter.hpp>
#include <nav_msgs/Path.h>
#include <iterator>
#include <stdio.h>
#include <math.h>

#define pair_double std::pair<double, double>


// Стягивание точек по вектору, относительно суммы из двух соседних точек
void tightening(nav_msgs::Path* path)
{
	double radius;
	pair_double line;
	pair_double *pr_point = new pair_double{};
	pair_double *nx_point = new pair_double{};
	pair_double *ourpoint = new pair_double{};
	
	pair_double *potential_vector1 = new pair_double{};
	pair_double *potential_vector2 = new pair_double{};
	geometry_msgs::PoseStamped *iter = new geometry_msgs::PoseStamped();

	std::vector<pair_double> out_vec;
	
	for(int i = 0 ; i < path->poses.size() -2; i ++)
		out_vec.push_back({0, 0});
	int i = 1;
	do
	{
		*iter = path->poses[i];

		*pr_point = {((*(iter - 1))).pose.position.x, (*(iter - 1)).pose.position.y};
		*nx_point = {((*(iter + 1))).pose.position.x, (*(iter + 1)).pose.position.y};
		*ourpoint = {(*iter).pose.position.x, (*iter).pose.position.y};

		*potential_vector1 =
			pair_double{ourpoint->first - pr_point->first, ourpoint->second - pr_point->second};
		*potential_vector2 =
			pair_double{nx_point->first - ourpoint->first, nx_point->second - ourpoint->second};

		out_vec[i] = {(potential_vector1->first + potential_vector2->first),
					(potential_vector2->second + potential_vector2->second)};

		
		i++;
	} while( i < path->poses.size() - 2);

	for(int i = 0 ; i < path->poses.size() -2; i ++)
	{
		path->poses[i].pose.position.x += out_vec[i].first;
		path->poses[i].pose.position.y += out_vec[i].second;	
	
	}
	return;
}

// Провекра на отработку скорости для остановки пробуксовки - костыли
void check_zero_speed(nav_msgs::Path* path, bool& move, pair_double old_pose)
{
	move = true;
	if (CalcDistance(transformPose(path->poses[0]), 
		transformPose(path->poses[ path->poses.size() - 1 ])) < 0.01)
		{
			move = false;
			return;
		}
	pair_double start = {path->poses[0].pose.position.x, path->poses[0].pose.position.y};
	
	double delta_y = path->poses[0].pose.position.y - path->poses[1].pose.position.y;
	double delta_x = path->poses[0].pose.position.x - path->poses[1].pose.position.x;
	
	double min_delta =  delta_x > delta_y ? delta_y: delta_y;
	bool move_no_speed = delta_y < 0.05 ? true : false;

	if ( CalcDistance( transformPose(path->poses[0]), old_pose) < 0.02 && move_no_speed)
	{
		ROS_INFO_THROTTLE(1," Zero speed ! ");
		move = false;
		return;
	}
}

void dynamic_inflation(double& inflation_radius, double speed, double Upper_level, double Lower_level)
{
	inflation_radius = (Upper_level - Lower_level) * speed;
	inflation_radius += Lower_level;   
	inflation_radius = inflation_radius > Upper_level ? Upper_level : inflation_radius;
}

std::vector<pair_double> appro_point(std::vector<pair_double> points, int index, int max_size)
{
	if( points.size() <= 4)
		{
			std::vector<pair_double> out = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
			return out;
		}
	/* TODO: Implement linear distance between 4 points */ 
	std::vector<pair_double> out = {points[ int((index % max_size )* 0.25) ], points[int((index % max_size )* 0.5 )],
		points[int((index % max_size )* 0.75 )], points[(int)(index % max_size )]};

	return out;
}



/* TODO: Сделать нормальное предсказние, выделив в отедельный проект*/
// Основано на построение сплайна, который является функции апроксимации пути робота
void PathPredictions(std::vector<pair_double>& path,  int &Index)
{
	int max_size = 50; // it is for path
	if (path.size() < 25 )
		{
			return;
		}
	std::vector<pair_double> appro_points = appro_point(path, Index, max_size);
	pair_double function[4];
	pair_double fun[4];
	double s = 0.5;
	// B - spline
	double M[4][4] =  { {-s,  2-s, s-2,    s}, 
						{2*s, s-3, 3-2*s, -s},
						{-s,  0,   s,      0},
						{0,   1,   0,      0} };
	for (int i = 0; i < 4; i++)
	{
		pair_double sum = {0, 0};
		for (int j = 0; j< 4; j++)
		{
			sum.first += M[j][i] * function[i].first;
			sum.second += M[j][i] * function[i].second;			
		}
			function[i].first = sum.first; 
			function[i].second = sum.second; 
	}

	for (int i = 0; i < 4; i++)
	{
		pair_double sum = {0, 0};
		for(int j = 0; j < 4; j++)
		{
			sum.first += appro_points[j].first * function[i].first;	
			sum.second += appro_points[j].second * function[i].second;		
				
		}
		fun[i].first = sum.first;
		fun[i].second = sum.second; 
	}

	
	double t_x = path[Index % max_size].first;
	double t_y = path[Index % max_size].second;
	
	path[++Index % max_size].first = pow(fun[0].first, 3) * (t_x + 3)+ 
		pow(fun[1].first, 2) * (t_x + 3) + fun[2].first * (t_x + 3)+ fun[3].first;

	path[++Index % max_size].second = pow(fun[0].second, 3) * (t_y + 3) +  
		pow(fun[1].second, 2) * (t_y  + 3) + fun[2].second * (t_y + 3) + fun[3].second;	
	
}

//  публикация для дебага в RVIZ
void visualisation_path_prediction(std::vector<pair_double>& path, ros::Publisher pub) 
{
	nav_msgs::Path output_path;
	if (path.size() != 0)
	{
		output_path.poses.resize( path.size() );
		for( int i = 0 ;i < path.size() - 1; i++)
		{
			output_path.poses[i].pose.position.x = path[i].first;
			output_path.poses[i].pose.position.y = path[i].second; 	
		}
	}
	pub.publish(output_path);
}
#endif