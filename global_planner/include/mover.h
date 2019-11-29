#ifndef MOVER_H
#define MOVER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

class Mover
{
  public:
    ros::Publisher accel_pub;
    ros::Publisher odom_pub;
    ros::Publisher real_pub;

    Mover(float _loop_rate = 10.0, float _speed = 1)
    {
        loop_rate = _loop_rate;
        speed = _speed;
    }
    void callbackLpSpeed(const geometry_msgs::Pose2D &msg)
    {
        if (check_speed_msg(msg))
        {
            speed_msg = msg;
        }
    }
    void callbackSpeed(const geometry_msgs::Pose2D &msg)
    {
        speed_msg = msg;
    }
    void callbackPathFull(const nav_msgs::Path &msg)
    {
        path_full = msg;
    }
    void callbackPathSimple(const nav_msgs::Path &msg)
    {
        path_simple = msg;
    }
    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg)
    {
        current_pose[0] = msg.pose.pose.position.x;
        current_pose[1] = msg.pose.pose.position.y;
        ROS_INFO("update current_pose %f %f", current_pose[0], current_pose[1]);
    }
    float moveFull()
    {
        return move(path_full, loop_rate, speed);
    }
    float moveSimple()
    {
        return move(path_simple, loop_rate, speed);
    }
    float moveSpeed()
    {
        double move_vect[3] = {speed_msg.x, speed_msg.y, speed_msg.theta};
        // ROS_INFO("loop_rate %f speed %f %f",loop_rate, move_vect[0], move_vect[1]);        
    //    ROS_INFO("size %i",sizeof(move_vect)/sizeof(move_vect[0])); 
        rotate_vector(move_vect,current_pose[2]);       
        for (int i = 0; i < sizeof(move_vect)/sizeof(move_vect[0]); i++)
        {
            move_vect[i] /= loop_rate;
            current_pose[i] += move_vect[i];
        }
        nav_msgs::Odometry odom_msg;
        array_to_odom(current_pose, odom_msg);
        // odom_msg.pose.pose.orientation = angle_goal;
        odom_pub.publish(odom_msg);
        real_pub.publish(odom_msg);
    }

  private:
    nav_msgs::Path path_full;
    nav_msgs::Path path_simple;
    geometry_msgs::Pose2D speed_msg;
    double current_pose[3] = {0.2, 0.2, 0.0};
    float loop_rate = 10.0;
    float speed;

    float move(const nav_msgs::Path &_path, const float &loop_rate, const float &speed)
    {
        nav_msgs::Path path = _path;
        ROS_INFO("path_size = %i", (int)path.poses.size());
        if (path.poses.size() < 1)
        {
            return 0;
        }
        float target_pose[2];
        target_pose[0] = static_cast<float>(path.poses[path.poses.size() - 2].pose.position.x);
        target_pose[1] = static_cast<float>(path.poses[path.poses.size() - 2].pose.position.y);
        geometry_msgs::Quaternion angle_goal = path.poses[path.poses.size() - 2].pose.orientation;
        target_pose[2] = getYawFromQuartenion(path.poses[path.poses.size() - 2].pose.orientation);

        float distance_to_target = distance(current_pose, target_pose);
        float move_distance = speed / loop_rate;
        move_distance = move_distance > distance_to_target ? distance_to_target : move_distance;

        float k = distance_to_target > 0.01 ? move_distance / distance_to_target : 0;
        ROS_INFO("current_pose= %f %f", current_pose[0], current_pose[1]);
        current_pose[0] = current_pose[0] + (target_pose[0] - current_pose[0]) * k;
        current_pose[1] = current_pose[1] + (target_pose[1] - current_pose[1]) * k;
        ROS_INFO("target_pose= %f %f", target_pose[0], target_pose[1]);
        ROS_INFO("distance_to_target= %f", distance_to_target);
        ROS_INFO("move_pose= %f %f \n", current_pose[0], current_pose[1]);

        nav_msgs::Odometry odom_msg;
        array_to_odom(current_pose, odom_msg);

        odom_pub.publish(odom_msg);
        real_pub.publish(odom_msg);
        return move_distance;
    }
    void rotate_vector(double *vect, float angle)
    {
        double tmp[2];
        tmp[0] = vect[0] * cos(angle) - vect[1] * sin(angle);
        tmp[1] = vect[0] * sin(angle) + vect[1] * cos(angle);
        vect[0] = tmp[0];
        vect[1] = tmp[1];
    }
    bool check_speed_msg(const geometry_msgs::Pose2D &msg)
    {// check correct msg data
        bool correct = true;
        float min = -10.0;
        float max = 10.0;
        correct = ((msg.x > min) && (msg.x < max));
        correct = correct && ((msg.y > min) && (msg.y < max));
        return correct;
    }
    float getYawFromQuartenion(const geometry_msgs::Quaternion &qMsg)
    {
        tf::Quaternion q;
        tf::quaternionMsgToTF(qMsg, q);
        tf::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        return static_cast<float>(y);
    }
    float distance(const double *p1, const float *p2)
    {
        return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
    }
    void array_to_odom( double *array, nav_msgs::Odometry &odom)
    {
        odom.header.frame_id = "map";
        odom.child_frame_id = "map";
        ros::Time stamp = ros::Time::now();
        odom.header.stamp.nsec = stamp.nsec;
        odom.header.stamp.sec = stamp.sec;
        // array[0] = array[0] > 200 ? 199 : array[0];
        // array[1] = array[1] > 300 ? 299 : array[1];
        // array[0] = array[0] < 0 ? 1 : array[0];
        // array[1] = array[1] < 0 ? 1 : array[1];

        odom.pose.pose.position.x = array[0];
        odom.pose.pose.position.y = array[1];
        odom.pose.pose.position.z = 0;
        tf2::Quaternion q;
        q.setEuler(0, 0, array[2]);
        // ROS_ERROR("");
        q.normalize();
        odom.pose.pose.orientation.w = q.getW();
        odom.pose.pose.orientation.x = q.getX();
        odom.pose.pose.orientation.y = q.getY();
        odom.pose.pose.orientation.z = q.getZ();
    }
};

#endif
