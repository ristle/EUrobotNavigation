
#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <fichi.hpp>

class Planner
{
  public:
    Planner(double inflation_radius_, std::string frame_id_)
    {    
      inflation_radius = inflation_radius_;
      for (int i = 0; i < 300; i++)
      {
        for(int j = 0; j < 200; j++)
          our_map[i][j] = 3000.0;
      }
      OurPath->poses.resize(50);
      
      geometry_msgs::Pose pose;
      
      pose.position.x = 0.0;
      pose.position.y = 0.0;
    
      for( int i = 0; i < 50; i++)
      {
        OurPath->poses[i].pose = pose;
      }
      
      frame_id = frame_id_;
    }
    virtual ~Planner() {  } 
    virtual void UpdatePath() = 0;

    ros::Publisher move_pub;	
    ros::Publisher BigEnemyPub;
    ros::Publisher LittleEnemyPub;
    ros::Publisher local_path_publisher;
    
  protected:

    // virtual double Calculate_Path_Len();

    // Рекурсивное добавление точек с определенной частотой
    nav_msgs::Path* recursive_path(nav_msgs::Path *path, std::pair<double, double> start, std::pair<double, double> end,
              double epsilon, int &index)
    {
      if (CalcDistance(start, end) < (epsilon) || path->poses.size() > 200)
        return path;

      double start_x = (start.first + end.first) / 2.0;
      double start_y = (start.second + end.second) / 2.0;

      index = find_out(path, start);

      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = start_x;
      pose.pose.position.y = start_y;
      path->poses.insert(path->poses.begin() + index, pose);

      recursive_path(path, start, std::pair<double, double>{start_x, start_y}, epsilon, index);
      recursive_path(path, std::pair<double, double>{start_x, start_y}, end, epsilon, index);
    }
    // Найти индекс
    int find_out(nav_msgs::Path *path, pair_double point)
    {
      int index = 0;
      for (int i = 0; i < path->poses.size(); i++)
        if (path->poses[i].pose.position.x == point.first &&
          path->poses[i].pose.position.y == point.second)
          return ++i;
      return index;
    }
    void add_in_path(nav_msgs::Path* Path, geometry_msgs::PoseStamped pose, int& Index)
    {
      Path->poses[Index % max_size] = pose;
      Index++;
    }


    double inflation_radius;
    double our_map[300][200];

	  int max_size = 49;
    
    std::string frame_id;

	  nav_msgs::Path *path = new nav_msgs::Path;
	  nav_msgs::Path *OurPath = new nav_msgs::Path; 
};

#endif