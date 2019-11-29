#ifndef COST_MAP_SERVER_H
#define COST_MAP_SERVER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <cost_map_ros/cost_map_ros.hpp>
#include <tf/transform_datatypes.h>
#include <string>

class CostMapServer
{
  public:
    ros::Publisher static_publisher;
    ros::Publisher obstacle_publisher;
    ros::Publisher inflation_publisher;
    ros::Publisher inflation_half_publisher;
    ros::Publisher cost_map_publisher;
    ros::Publisher paint_publisher;
    ros::Publisher obstacle_inflation_publisher;
    CostMapServer(
        std::string static_costs_image_resource_name,
        double big_robot_size_,
        double little_robot_size_,
        double inscribed_radius_,
        double inflation_radius_,
        double inflation_exponential_rate_,
        double min_diff_points_,
        std::string side_)
    {
       
        big_robot_size = big_robot_size_;
        little_robot_size = little_robot_size_;
        inscribed_radius = inscribed_radius_;
        inflation_radius = inflation_radius_;
        inflation_exponential_rate = inflation_exponential_rate_;
        min_diff_points = min_diff_points_;
        bigEnemyTimer = ros::Time::now().toSec();
        littleEnemyTimer = ros::Time::now().toSec();
        myBigFriendTimer = ros::Time::now().toSec(); 
        myLittleFriendTimer = ros::Time::now().toSec();
        /****************************************
        ** Load cost_map from image
        ****************************************/
        std::shared_ptr<cost_map::LoadImageBundle> loader;
        loader = std::make_shared<cost_map::LoadImageBundle>(static_costs_image_resource_name);
        costmap = *(loader->cost_map);
        /****************************************
        ** Create sub layers
        ****************************************/
        costmap.add("inflation_layer", cost_map::FREE_SPACE);
        costmap.add("obstacle_layer", cost_map::FREE_SPACE);           
        costmap.add("static_inflation_layer", cost_map::FREE_SPACE);              // private
        inflation_computer = new cost_map::ROSInflationComputer(inscribed_radius, inflation_exponential_rate);

        draw_side_specific_obstacle(side_);
        inflate("static_layer", "static_inflation_layer",
                inflation_radius,
                *inflation_computer,
                costmap);
        
        Inflate();                                                     // need to init layers

        free_space_point.pose.position.x = 1.5;
        free_space_point.pose.position.y = 0.35;
    }
    void callbackBigEnemy(const geometry_msgs::PoseStamped &msg)
    {
        if (IsIncorrectPoint(msg))
        {
            return;
        }
        bigEnemyNew = msg;
        bigEnemyTimer = ros::Time::now().toSec();
    }
    void callbackLittleEnemy(const geometry_msgs::PoseStamped &msg)
    {
        if (IsIncorrectPoint(msg))
        {
            return;
        }
        littleEnemyNew = msg;
        littleEnemyTimer = ros::Time::now().toSec();
    }
    void callbackMyBigFriend(const geometry_msgs::PoseStamped &msg)
    {
        if (IsIncorrectPoint(msg))
        {
            return;
        }
        myBigFriendNew = msg;
        myBigFriendTimer = ros::Time::now().toSec();
    }
    void callbackMyLittleFriend(const geometry_msgs::PoseStamped &msg)
    {
        if (IsIncorrectPoint(msg))
        {
            return;
        }
        myLittleFriendNew = msg;
        myLittleFriendTimer = ros::Time::now().toSec();
    }
    void PublishStaticLayer()
    {
        nav_msgs::OccupancyGrid msg;
        cost_map::toOccupancyGrid(costmap, "static_layer", msg);
        static_publisher.publish(msg);
    }
    void UpdateMap()
    {
        Delete_unuse_obstacle();

        double watchdog = ros::Time::now().toSec();
        
        if ((watchdog - myLittleFriendTimer) < 5){
            UpdateObstaclePosition(myLittleFriend, myLittleFriendNew);
            UpdateObstacleOnly(myLittleFriendNew, "obstacle_layer", little_robot_size);
        }

        if ((watchdog - myBigFriendTimer) < 5){
            UpdateObstaclePosition(myBigFriend, myBigFriendNew);
            UpdateObstacleOnly(myBigFriendNew, "obstacle_layer", little_robot_size);
        }

        if ((watchdog - littleEnemyTimer) < 5){
            UpdateObstaclePosition(littleEnemy, littleEnemyNew);
            UpdateObstacleOnly(littleEnemyNew, "obstacle_layer",little_robot_size);
        }

        if ((watchdog - bigEnemyTimer) < 5){
            UpdateObstaclePosition(bigEnemy, bigEnemyNew);
            UpdateObstacleOnly(bigEnemyNew, "obstacle_layer", little_robot_size);
        }

        myLittleFriendNew=myLittleFriend;
        myBigFriendNew=myBigFriend;
        littleEnemyNew=littleEnemy;
        bigEnemyNew=bigEnemy;

        Inflate();
        PublishChanges();

    }

    void UpdateObstacleOnly(
    const geometry_msgs::PoseStamped &p,
    std::string layer,
    double size_x, double size_y)
    {
        Draw(p, layer, size_x, size_y,cost_map::LETHAL_OBSTACLE);   
    }

    void UpdateObstacleOnly(
    const geometry_msgs::PoseStamped &p,
    std::string layer,
    double size_)
    {
        Draw(p, layer, size_, cost_map::LETHAL_OBSTACLE);   
    }

    void PublishDynamicLayer()
    {
        Inflate();
        PublishChanges();
    }
    void Delete_unuse_obstacle()
    {
        costmap["obstacle_layer"].setConstant(cost_map::FREE_SPACE);
    }

  private:
    double inscribed_radius;
    double inflation_radius;
    double inflation_exponential_rate;
    double big_robot_size; //TODO: remove hardcoded names
    double little_robot_size;
    double min_diff_points;
    double diff[3] = {0.0,0.0,0.0};
    double angle = 0.;
    double bigEnemyTimer, littleEnemyTimer, myBigFriendTimer, myLittleFriendTimer;
    geometry_msgs::PoseStamped bigEnemy, littleEnemy, myBigFriend, myLittleFriend;
    geometry_msgs::PoseStamped bigEnemyNew, littleEnemyNew, myBigFriendNew, myLittleFriendNew;
    cost_map::CostMap costmap;
    cost_map::Inflate inflate;
    cost_map::ROSInflationComputer* inflation_computer;
    geometry_msgs::PoseStamped free_space_point;
    double free_space_size = 0.7;

    void UpdateObstaclePosition(geometry_msgs::PoseStamped &p1,geometry_msgs::PoseStamped &p2)
    {
        // ROS_ERROR("p1 %f %f",p1.pose.position.x,p1.pose.position.y);
        // ROS_ERROR("p2 %f %f",p2.pose.position.x,p2.pose.position.y);
        geometry_msgs::PoseStamped p3 = p2;
        GetDiff(p1,p2,diff,angle);
        // ROS_ERROR("diff %f %f",diff[0],diff[1]);
        // ROS_ERROR("p3 %f %f",p3.pose.position.x,p3.pose.position.y);
        MovePose(p2,diff);
        p1=p3;
        // ROS_ERROR("p1 %f %f",p1.pose.position.x,p1.pose.position.y);
        return;
    }
    void GetDiff(const geometry_msgs::PoseStamped &p1,const geometry_msgs::PoseStamped &p2, double* diff,double& angle){
        double time_delta = p2.header.stamp.toSec() - p2.header.stamp.toSec(); 
        *diff = (p2.pose.position.x - p1.pose.position.x); //* cos(angle);
        *(diff+1) = (p2.pose.position.y - p1.pose.position.y);// * sin(angle);
        // diff[2] = 0;

    }
    void MovePose(geometry_msgs::PoseStamped &p2,const double* diff){
        p2.pose.position.x+=*diff;
        p2.pose.position.y+=*(diff+1);
        // p2.pose.position.x+=diff[2];
    }
    bool IsIncorrectPoint(const geometry_msgs::PoseStamped &p)
    {
        return ((p.pose.position.x <= 0) ||
                (p.pose.position.x >= 3) || //TODO: remove hardcoded size
                (p.pose.position.y <= 0) ||
                (p.pose.position.y >= 2));
    }
    bool IsPointsSimilar(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2, float min_diff)
    {
        return (fabs(p1.pose.position.x - p2.pose.position.x) < min_diff) &&
               (fabs(p1.pose.position.y - p2.pose.position.y) < min_diff);
    }
    void AddObstacle(
        const geometry_msgs::PoseArray &obstacle_array,
        std::string layer,
        double radius)
    {
        for (unsigned int iterator = 0; iterator < obstacle_array.poses.size(); iterator++)
        {
            Draw(obstacle_array.poses[iterator], layer, radius, cost_map::LETHAL_OBSTACLE);
        }
        Inflate();
        PublishChanges();
    }
    void RemoveObstacle(
        const geometry_msgs::PoseArray &obstacle_array,
        std::string layer,
        double radius)
    {
        for (unsigned int iterator = 0; iterator < obstacle_array.poses.size(); iterator++)
        {
            Draw(obstacle_array.poses[iterator], layer, radius, cost_map::FREE_SPACE);
        }
        Inflate();
        PublishChanges();
    }

    void RemoveObstacle(
        const geometry_msgs::PoseStamped obstacle,
        std::string layer,
        double radius)
    {
        Draw(obstacle, layer, radius, 3);
        Inflate();
        PublishChanges();
    }

    void UpdateObstacle(
        geometry_msgs::PoseStamped &old_obstacle,
        const geometry_msgs::PoseStamped &new_obstacle,
        std::string layer,
        double radius)
    {
        Draw(old_obstacle, layer, radius, cost_map::FREE_SPACE);
        old_obstacle = new_obstacle;
        Draw(new_obstacle, layer, radius, cost_map::LETHAL_OBSTACLE);
        Inflate();
        PublishChanges();
    }
    void UpdateObstacle(
        geometry_msgs::PoseStamped &old_obstacle,
        const geometry_msgs::PoseStamped &new_obstacle,
        std::string layer,
        double size_x,
        double size_y)
    {
        Draw(old_obstacle, layer, size_x, size_y, cost_map::FREE_SPACE);
        old_obstacle = new_obstacle;
        ROS_ERROR("update, x=%f", old_obstacle.pose.position.x);
        Draw(new_obstacle, layer, size_x, size_y, cost_map::LETHAL_OBSTACLE);
        Inflate();
        PublishChanges();
    }
    void UpdateObstacle(
        geometry_msgs::PoseArray &old_obstacle_array,
        const geometry_msgs::PoseArray &new_obstacle_array,
        std::string layer,
        double size_x,
        double size_y)
    {
        for (unsigned int iterator = 0; iterator < old_obstacle_array.poses.size(); iterator++)
        {
            Draw(old_obstacle_array.poses[iterator], layer, size_x, size_y, cost_map::FREE_SPACE);
        }
        old_obstacle_array = new_obstacle_array;
        if (old_obstacle_array.poses.size() > 0)
        {
            for (unsigned int iterator = 0; iterator < new_obstacle_array.poses.size(); iterator++)
            {
                Draw(new_obstacle_array.poses[iterator], layer, size_x, size_y, cost_map::LETHAL_OBSTACLE);
            }
        }
        Inflate();
        PublishChanges();
    }

    void UpdateObstacle(
        geometry_msgs::PoseArray &old_obstacle_array,
        const geometry_msgs::PoseArray &new_obstacle_array,
        std::string layer,
        double radius)
    {
        for (unsigned int iterator = 0; iterator < old_obstacle_array.poses.size(); iterator++)
        {
            Draw(old_obstacle_array.poses[iterator], layer, radius, cost_map::FREE_SPACE);
        }
        old_obstacle_array = new_obstacle_array;
        if (old_obstacle_array.poses.size() > 0)
        {
            for (unsigned int iterator = 0; iterator < new_obstacle_array.poses.size(); iterator++)
            {
                Draw(new_obstacle_array.poses[iterator], layer, radius, cost_map::LETHAL_OBSTACLE);
            }
        }
        Inflate();
        PublishChanges();
    }
    void Draw(
        const geometry_msgs::PoseStamped &obstacle,
        std::string layer,
        double radius,
        unsigned char value)
    {
        cost_map::Position center = {obstacle.pose.position.x, obstacle.pose.position.y};
        for (cost_map::CircleIterator iterator(costmap, center, radius);
             !iterator.isPastEnd(); ++iterator)
        {
            costmap.at(layer, *iterator) = value;
        }
    }
    void Draw(
        const geometry_msgs::PoseStamped &obstacle,
        std::string layer,
        double size_x,
        double size_y,
        unsigned char value)
    {
        cost_map::Position center = {obstacle.pose.position.x, obstacle.pose.position.y};
        grid_map::Polygon polygon;
        polygon.setFrameId(costmap.getFrameId());
        polygon.addVertex(grid_map::Position(center[0] - size_x / 2.0, center[1] + size_y / 2.0));
        polygon.addVertex(grid_map::Position(center[0] - size_x / 2.0, center[1] - size_y / 2.0));
        polygon.addVertex(grid_map::Position(center[0] + size_x / 2.0, center[1] - size_y / 2.0));
        polygon.addVertex(grid_map::Position(center[0] + size_x / 2.0, center[1] + size_y / 2.0));

        for (cost_map::PolygonIterator iterator(costmap, polygon);
             !iterator.isPastEnd(); ++iterator)
        {
            costmap.at(layer, *iterator) = value;
        }
    }



    void Draw(const geometry_msgs::Pose &obstacle, std::string layer, double size_x, double size_y, unsigned char value)
    {
        cost_map::Position center = {obstacle.position.x, obstacle.position.y};
        grid_map::Polygon polygon;
        polygon.setFrameId(costmap.getFrameId());
        polygon.addVertex(grid_map::Position(center[0] - size_x / 2.0, center[1] + size_y / 2.0));
        polygon.addVertex(grid_map::Position(center[0] - size_x / 2.0, center[1] - size_y / 2.0));
        polygon.addVertex(grid_map::Position(center[0] + size_x / 2.0, center[1] - size_y / 2.0));
        polygon.addVertex(grid_map::Position(center[0] + size_x / 2.0, center[1] + size_y / 2.0));
        for (cost_map::PolygonIterator iterator(costmap, polygon);
             !iterator.isPastEnd(); ++iterator)
        {
            costmap.at(layer, *iterator) = value;
        }
    }
    
    void Draw(const geometry_msgs::Pose &obstacle, std::string layer, double radius, unsigned char value)
    {
        cost_map::Position center = {obstacle.position.x, obstacle.position.y};
        for (cost_map::CircleIterator iterator(costmap, center, radius);
             !iterator.isPastEnd(); ++iterator)
        {
            costmap.at(layer, *iterator) = value;
        }
    }
    void draw_side_specific_obstacle(std::string side_){

        geometry_msgs::PoseStamped yellow_periodic_table;
        geometry_msgs::PoseStamped purple_periodic_table;
        yellow_periodic_table.pose.position.x = 0.225;
        yellow_periodic_table.pose.position.y = 1.25;
        purple_periodic_table.pose.position.x = 2.775;
        purple_periodic_table.pose.position.y = 1.25;
        double periodic_table_size_x = 0.45;
        double periodic_table_size_y = 0.9;

        geometry_msgs::PoseStamped yellow_scalyator;
        geometry_msgs::PoseStamped purple_scalyator;
        yellow_scalyator.pose.position.x = 1.350;
        yellow_scalyator.pose.position.y = 0.5;
        purple_scalyator.pose.position.x = 1.650;
        purple_scalyator.pose.position.y = 0.5;
        double scalyator_size_x = 0.3;
        double scalyator_size_y = 0.2;

        geometry_msgs::PoseStamped yellow_accelerator;
        geometry_msgs::PoseStamped purple_accelerator;
        yellow_accelerator.pose.position.x = 1.650;
        yellow_accelerator.pose.position.y = 1.9;
        purple_accelerator.pose.position.x = 1.350;
        purple_accelerator.pose.position.y = 1.9;
        double accelerator_size_x = 0.3;
        double accelerator_size_y = 0.2;

        if (side_ == "yellow")
        {
            Draw(purple_periodic_table, "static_layer", periodic_table_size_x, periodic_table_size_y, cost_map::LETHAL_OBSTACLE);
            Draw(purple_scalyator, "static_layer", scalyator_size_x, scalyator_size_y, cost_map::LETHAL_OBSTACLE);
            Draw(purple_accelerator, "static_layer", accelerator_size_x, accelerator_size_y, cost_map::LETHAL_OBSTACLE);
        }
        if(side_ == "purple")
        {
            Draw(yellow_periodic_table, "static_layer", periodic_table_size_x, periodic_table_size_y, cost_map::LETHAL_OBSTACLE);
            Draw(yellow_scalyator, "static_layer", scalyator_size_x, scalyator_size_y, cost_map::LETHAL_OBSTACLE);
            Draw(yellow_accelerator, "static_layer", accelerator_size_x, accelerator_size_y, cost_map::LETHAL_OBSTACLE);
        }

    }
    
    void Inflate()
    {
        Draw(free_space_point, "obstacle_layer", free_space_size,free_space_size, cost_map::FREE_SPACE);
        costmap["inflation_layer"] = costmap["static_inflation_layer"].cwiseMax(costmap["obstacle_layer"]);
    }

    void PublishChanges()
    {
        nav_msgs::OccupancyGrid msg;
        cost_map_msgs::CostMap cost_map_msg;
        if (obstacle_inflation_publisher.getNumSubscribers() > 0)
        {
            cost_map::toOccupancyGrid(costmap, "obstacle_layer", msg);
            obstacle_inflation_publisher.publish(msg);
        }
        if (inflation_publisher.getNumSubscribers() > 0)
        {
            cost_map::toOccupancyGrid(costmap, "inflation_layer", msg);
            inflation_publisher.publish(msg);
        }
        if (cost_map_publisher.getNumSubscribers() > 0)
        {
            cost_map::toMessage(costmap, cost_map_msg);
            cost_map_publisher.publish(cost_map_msg);
        }
    }
   };

#endif
