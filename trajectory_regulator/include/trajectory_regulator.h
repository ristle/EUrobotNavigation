#ifndef trajectory_regulator_H
#define trajectory_regulator_H

#include "global_planner/Enable.h"
#include "global_planner/Mode.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <true_modulo.h>

#include "lineiteratorOccypancy.hpp"
#include <PlannerTrajectory.hpp>

struct PathPlannerParams {
  std::string frame_id = "map";
  std::string path_layer = "inflation_layer";
  std::string path_force_layer = "obstacle_layer";
  double path_filter_epsilon = 0.05;
};

typedef struct {
  float p_k;     //П коэфициент
  float i_k;     //И коэфициент
  float d_k;     //Д коэфициент
  float target;  //Целевое значение
  float current; //Текущее (необходимо обновлять извне перед каждым расчетом)
  float prev_error; //Предыдущее значение ошибки (для Д регелятора)
  float sum_error; //Суммарная ошибка (для И регулятора)
  float max_sum_error; //Максимальная суммарная ошибка (что бы И регулятор не
                       //уходил до максимума если невозможно добиться требуемого
                       //значения)
  float max_output; //Максимальный выход, что бы поправка не выходила за рамки
  float min_output;
  float cut_output;
  float output; //Поправка, результат работы ПИД
  char pid_on; //Вкл/Выкл ПИД если выкл то output всегда равен 0, однако все
               //остальное продолжает расчитываться
  char pid_finish;
  float error_dir;
  float pid_error_end;
  float pid_output_end;
} PidStruct;
namespace grid_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

// designed to match those in costmap_2d/cost_values.h
const double LETHAL_OBSTACLE = 0.5;

} // namespace grid_map

void printPath(nav_msgs::Path &p) {
  for (auto pose : p.poses) {
    printf("x:%f y:%f \n", pose.pose.position.x, pose.pose.position.y);
  }
  return;
}

class TrajectoryRegulator {
public:
    ros::Publisher move_pub;
    ros::Publisher is_stop_pub;
    ros::Publisher move_debug_pub;

    TrajectoryRegulator(float loop_rate_, float _max_speed = 0.1,
                        float _max_acc = 0.02, float _max_braking = 0.01,
                        float _max_rotation = 3.0, float _min_speed = 0.05) {
        costmap = cost_map::CostMap();
        max_speed = _max_speed;
        max_acc = _max_acc;
        max_braking = _max_braking;
        max_rotation = _max_rotation;
        loop_rate = loop_rate_;
        min_speed = _min_speed;
        initRegulators(&_angle_pid, 3.0, 0.02, 1.2);
    }

    void callbackCostMap(const cost_map_msgs::CostMap::ConstPtr &msg) {
        cost_map::fromMessage(*msg, costmap);
        is_init_state[2] = true;
    }

    void callbackRealCorr(const nav_msgs::Odometry &msg) {
        last_pose[0] = current_pose[0];
        last_pose[1] = current_pose[1];
        last_pose[2] = current_pose[2];
        time_last.nsec = time_current.nsec;
        time_last.sec = time_current.sec;
        current_pose[0] = msg.pose.pose.position.x;
        current_pose[1] = msg.pose.pose.position.y;
        current_pose[2] = getYawFromQuartenion(msg.pose.pose.orientation);

        Current[0].velocity = current_pose[0] - last_pose[0];
        Current[1].velocity = current_pose[1] - last_pose[1];
        Current[0].acceleration =
                Current[0].velocity /
                ((ros::Time::now().nsec - time_last.nsec) / 1000.);
        Current[1].acceleration =
                Current[1].velocity /
                ((ros::Time::now().nsec - time_last.nsec) / 1000.);

        time_current = msg.header.stamp;
        current_speed = calc_speed(current_vect, current_pose, last_pose,
                                   time_current, time_last);
        is_init_state[0] = true;
    }

    void callbackReal(const nav_msgs::Odometry &msg) {
        last_pose[0] = current_pose[0];
        last_pose[1] = current_pose[1];
        last_pose[2] = current_pose[2];
        time_last.nsec = time_current.nsec;
        time_last.sec = time_current.sec;
        current_pose[0] = msg.pose.pose.position.x;
        current_pose[1] = msg.pose.pose.position.y;
        current_pose[0] = float(int(current_pose[0] * 100) / 100.);
        current_pose[1] = float(int(current_pose[1] * 100) / 100.);
        current_pose[2] = getYawFromQuartenion(msg.pose.pose.orientation);
        Current[0].velocity = double(current_pose[0] - last_pose[0]) * 100;
        Current[1].velocity = double(current_pose[1] - last_pose[1]) * 100;
        Current[0].acceleration =
                (Current[0].velocity /
                 ((ros::Time::now().nsec - time_last.nsec) / 10000.));
        Current[1].acceleration =
                (Current[1].velocity /
                 ((ros::Time::now().nsec - time_last.nsec) / 10000.));
        time_current = msg.header.stamp;
        current_speed = calc_speed(current_vect, current_pose, last_pose,
                                   time_current, time_last);
        is_init_state[0] = true;
    }

    void callbackPathSimple(const nav_msgs::Path &msg) {
        local_path = msg;
        is_init_state[1] = true;
        index = local_path.poses.size() - 2;
    }

    bool callbackSrvEnable(global_planner::Enable::Request &req,
                           global_planner::Enable::Response &res) {
        enable = req.enable.data;
        ROS_INFO("got Enable move: %s", enable == true ? "true" : "false");
        return true;
    }

    bool callbackSrvMode(global_planner::Mode::Request &req,
                         global_planner::Mode::Response &res) {
        obstacle = req.obstacle.data;
        max_speed = req.speed.data < 0 ? max_speed : req.speed.data;
        max_acc = req.acceleration.data < 0 ? max_acc : req.acceleration.data;
        max_braking = req.braking.data < 0 ? max_braking : req.braking.data;
        ROS_INFO("got Mode\n obstacle: %s\n speed: %f\n acceleration: %f\n "
                 "braking: %f\n",
                 obstacle == true ? "On" : "Off", max_speed, max_acc, max_braking);
        return false;
    }

    bool is_init() {
        return (is_init_state[0] and is_init_state[1] and is_init_state[2]);
    }

    float move() {
        float next_pose[3] = {0, 0, 0};
        float move_vect[3] = {0, 0, 0};
        float path_distance = 0;
        std::string check_layer = obstacle ? "inflation_layer" : "obstacle_layer";
        if (local_path.poses.size() < 1) {
            publish_move(move_vect, move_pub);
            publish_debug(move_vect, move_debug_pub);
            ROS_INFO_THROTTLE(1, "[tr] path is 0");
            return -1;
        }
        if (check_path_unreachable(local_path, 4, costmap, check_layer)) {
            publish_move(move_vect, move_pub);
            publish_debug(move_vect, move_debug_pub);
            ROS_INFO_THROTTLE(1, "[tr] path unreachable");
            return -1;
        }
        // ROS_ERROR("SHIT:  %f", sqrt(pow(local_path.poses[0].pose.position.x -
        // local_path.poses[local_path.poses.size()-1].pose.position.x,2 )
        // + pow(local_path.poses[0].pose.position.y -
        // local_path.poses[local_path.poses.size()-1].pose.position.y, 2 )));
        ROS_ERROR("[tr] Pose START: %f %f  \t  GOAL: %f %f",
                  local_path.poses[0].pose.position.x,
                  local_path.poses[0].pose.position.y,
                  local_path.poses[local_path.poses.size() - 1].pose.position.x,
                  local_path.poses[local_path.poses.size() - 1].pose.position.y);

        if (sqrt(pow(local_path.poses[0].pose.position.x -
                     local_path.poses[local_path.poses.size() - 1]
                             .pose.position.x,
                     2) +
                 pow(local_path.poses[0].pose.position.y -
                     local_path.poses[local_path.poses.size() - 1]
                             .pose.position.y,
                     2)) < 0.06) {
            publish_move(move_vect, move_pub);
            publish_debug(move_vect, move_debug_pub);
            ROS_INFO_THROTTLE(1, "[tr] goal reached");
            return -1;
        }
        if (local_path.poses.size() != 2) {
            // 0.00123048, -5.86741e-09 Simples
            // TODO: CHECK THE DIRECTION OF THE OUTPUT VECTORS!
            Output[0].velocity = 0.1;                         // Average velocity
            Output[0].acceleration = Output[0].velocity / 17; // Randomly velocity

            Output[1].velocity = 0.1;                         // Average velocity
            Output[1].acceleration = Output[0].velocity / 17; // Randomly acceleration
        }

        ROS_ERROR("[tr] Poses %f %f \t Output %f %f", Current[0].velocity,
                  Current[0].acceleration, Output[1].velocity,
                  Output[1].acceleration);
        trajectory->GetVectorFromTrajectory(local_path, params.path_layer, Current,
                                            Output, (move_vect));

        get_path_dist(path_distance, local_path, current_pose);
        diff(move_vect, current_pose, local_path, path_distance);

        double magic_value = -(current_pose[2] + move_vect[2] / loop_rate);

        rotate_vector(move_vect, magic_value);

        _angle_pid.target = next_pose[2];
        _angle_pid.current = current_pose[2];

        pidCalc(&_angle_pid);

        move_vect[2] = _angle_pid.output;

        if (!enable) {
            move_vect[0] = 0.0;
            move_vect[1] = 0.0;
            move_vect[2] = 0.0;
        }
        // move_vect[2]+=(current_vect[2]/2);
        // current_vect[2]=move_vect[2];
        publish_move(move_vect, move_pub);
        publish_debug(move_vect, move_debug_pub);
        return 0;
    }

private:
    PathPlannerParams params;
    cost_map::CostMap costmap;
    nav_msgs::Path local_path;
    float current_pose[3] = {0.1, 0.1, 0.0};
    float last_pose[3];

    ros::Time time_current, time_last;
    float max_speed, max_acc, max_braking;
    float max_acceleration, min_speed, max_rotation;
    double average_velocity = 0.5;
    int frequency = 30;

    bool enable = true;
    bool obstacle = true;

    bool is_init_state[3] = {false, false, false};
    float loop_rate;
    int last_point = 0, next_point = 0;
    float current_speed = 0;
    float current_vect[3] = {0, 0, 0};
    float decrease_k = 0.3;
    float decrease_dist = 0.1;
    float Resolution = 0.01;
    bool magic_value = true;
    int index = 2;
    int length_x = 3, length_y = 2;


    PlannerTrajectory *trajectory = new PlannerTrajectory(
            &costmap, frequency, (double) max_speed, average_velocity);
    PidStruct _angle_pid;

    State Current[2]{State(0., 0.), State(0., 0.)};
    State Output[2]{State(0., 0.), State(0., 0.)};

    void norm(float *out, float max_value) {
        out[2] = out[2] > max_rotation ? max_rotation : out[2];
        out[2] = out[2] < -max_rotation ? -max_rotation : out[2];
    }

    bool decrease_speed_near_points(double &new_max_speed,
                                    const float *const next_pose,
                                    const float *const current_pose,
                                    const float decrease_k,
                                    const float decrease_dist) {
        new_max_speed = distance(next_pose, current_pose) < decrease_dist
                        ? new_max_speed * decrease_k
                        : new_max_speed;
    }

    bool check_path_unreachable(nav_msgs::Path in_path,
                                const size_t n_first_points,
                                cost_map::CostMap &costmap,
                                std::string layer) {
        size_t n_checks = in_path.poses.size() < n_first_points
                          ? in_path.poses.size()
                          : n_first_points;

        for (int it = in_path.poses.size() - 1; n_checks > 0; n_checks--, it--) {
            double x = in_path.poses[it].pose.position.x;
            double y = in_path.poses[it].pose.position.y;
            double lengtX = length_x;
            double lengtY = length_y;

            x = x > lengtX ? lengtX - 1 : x;
            y = y > lengtY ? lengtY - 1 : y;
            x = x < 0.01 ? 0.01 : x;
            y = y < 0.01 ? 0.01 : y;

            double resolution = Resolution;
            // check that point on map
            if (x < resolution || y < resolution || x > (lengtX - resolution) ||
                y > (lengtY - resolution)) {
                return true;
            }

            cost_map::Position p = cost_map::Position(x, y);
            cost_map::Index in;
            costmap.getIndex(p, in);
            double cost = costmap.at(layer, in);

            if (cost > cost_map::LETHAL_OBSTACLE) {
                return true;
            }
        }
        return false;
    }

    void initRegulators(PidStruct *pidStruct, float kp, float ki, float kd) {
        // PID Regulator
        // settings________________________________________________________
        int i = 0;

        pidStruct->p_k = kp; // 5.0
        pidStruct->i_k = ki; // 1
        pidStruct->d_k = kd; // 0.5
        pidStruct->pid_on = 1;
        pidStruct->pid_error_end = 3;
        pidStruct->pid_output_end = 1000;
        pidStruct->max_sum_error = 3.0;
        pidStruct->max_output = 3;
        pidStruct->min_output = 0;
        pidStruct->sum_error = 0;
        pidStruct->prev_error = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    void pidCalc(PidStruct *pid_control) {
        float error, dif_error;
        error = pid_control->target - pid_control->current;
        error = angle_err(pid_control->current, pid_control->target);
        dif_error = error - pid_control->prev_error;
        pid_control->prev_error = error;
        pid_control->sum_error += error;

        if (pid_control->sum_error > pid_control->max_sum_error)
            pid_control->sum_error = pid_control->max_sum_error;
        if (pid_control->sum_error < -pid_control->max_sum_error)
            pid_control->sum_error = -pid_control->max_sum_error;

        if (pid_control->pid_on) {
            float _integral =
                    fabs(error) < 0.015 ? 0 : (pid_control->i_k * pid_control->sum_error);
            pid_control->output = ((float) (pid_control->p_k * error) + _integral +
                                   (pid_control->d_k * dif_error));

            if (pid_control->output > pid_control->max_output)
                pid_control->output = pid_control->max_output;
            else if (pid_control->output < -pid_control->max_output)
                pid_control->output = -pid_control->max_output;

            if (pid_control->output < pid_control->min_output &&
                pid_control->output > -pid_control->min_output)
                pid_control->output = 0;

            if ((pid_control->output <= pid_control->pid_output_end) &&
                (pid_control->output >= -pid_control->pid_output_end) &&
                (error <= pid_control->pid_error_end) &&
                (error >= -pid_control->pid_error_end))
                pid_control->pid_finish = 1;
            else
                pid_control->pid_finish = 0;
        } else {
            pid_control->output = 0;
            pid_control->pid_finish = 0;
        }
    }

    bool accel_limitation(float *move_vect, const float max_acc,
                          const float current_speed, float *current_vect) {
        float diff_vect[3] = {0, 0, 0};
        diff_vect[0] = fabs(move_vect[0]) - fabs(current_vect[0]);
        diff_vect[1] = fabs(move_vect[1]) - fabs(current_vect[1]);

        float max_diff = diff_vect[0] > diff_vect[1] ? diff_vect[0] : diff_vect[1];
        if (max_diff > max_acc) {
            norm(move_vect, (current_speed + max_diff));
            return true;
        }
        return false;
    }

    bool get_stop_state(float *move_vect) {
        if (move_vect) // zaebis`
            return false;
    }

    void get_path_dist(float &path_distance, const nav_msgs::Path &in_path,
                       const float *current_pose) {
        path_distance = 0;
        float pose1[2] = {current_pose[0], current_pose[1]};
        float pose2[2] = {current_pose[0], current_pose[1]};
        for (int i = in_path.poses.size() - 1; i >= 0; i--) {
            pose2[0] = in_path.poses[i].pose.position.x;
            pose2[1] = in_path.poses[i].pose.position.y;
            path_distance += distance(pose1, pose2);
            pose1[0] = pose2[0];
            pose1[1] = pose2[1];
        }
    }

    void get_next_pose(float *target_pose, const nav_msgs::Path &in_path) {
        size_t path_point = (int) in_path.poses.size() - 2;
        target_pose[0] = in_path.poses[path_point].pose.position.x;
        target_pose[1] = in_path.poses[path_point].pose.position.y;
        target_pose[2] =
                getYawFromQuartenion(in_path.poses[path_point].pose.orientation);
    }

    void get_next_pose3(float *target_pose, const nav_msgs::Path &in_path) {
        size_t path_point = (int) in_path.poses.size() - 3;
        target_pose[0] = in_path.poses[path_point].pose.position.x;
        target_pose[1] = in_path.poses[path_point].pose.position.y;
        target_pose[2] =
                getYawFromQuartenion(in_path.poses[path_point].pose.orientation);
    }

    void get_next_pose2(float *target_pose, const float *const current_pose,
                        const nav_msgs::Path &in_path,
                        const cost_map::CostMap &costmap,
                        const std::string layer) {
        size_t path_size = in_path.poses.size() - 1;
        bool line_is_free = false;
        int minimal_cost = 3;

        for (size_t it = 0; it < path_size; it++) {
            line_is_free = lineOfSight(in_path.poses[it].pose, current_pose, costmap,
                                       layer, minimal_cost);
            // ROS_INFO("line %i",line_is_free);
            if (line_is_free) {
                target_pose[0] = in_path.poses[it].pose.position.x;
                target_pose[1] = in_path.poses[it].pose.position.y;
                target_pose[2] =
                        getYawFromQuartenion(in_path.poses[it].pose.orientation);

                // ROS_INFO("got point %i in %i",it+1,in_path.poses.size());
                return;
            }
        }
        size_t path_point = (int) in_path.poses.size() - 2;
        target_pose[0] = in_path.poses[path_point].pose.position.x;
        target_pose[1] = in_path.poses[path_point].pose.position.y;
        target_pose[2] =
                getYawFromQuartenion(in_path.poses[path_point].pose.orientation);
    }

    size_t get_nearest_position(const nav_msgs::Path &in_path,
                                const float *current_pose) {
        float dist = 9999999999999;
        float pose[3];
        float point_from_path[2];
        size_t position;
        size_t path_size = (int) in_path.poses.size();
        for (size_t i = 0; i < path_size; i++) {
            point_from_path[0] = in_path.poses[i].pose.position.x;
            point_from_path[1] = in_path.poses[i].pose.position.y;
            float newdist = distance(point_from_path, current_pose);
            if (newdist < dist) {
                dist = newdist;
                position = i;
            }
        }
        return position;
    }

    void norm_min_speed(float *out, float min_value) {
        float max = fabs(out[0]) > fabs(out[1]) ? out[0] : out[1];
        if (fabs(max) > min_value) {
            return;
        }
        float k = fabs(max / min_value);
        k = k == 0 ? 1 : k;
        out[0] /= k;
        out[1] /= k;
    }

    double CalcDistance(std::pair<double, double> start,
                        std::pair<double, double> end) {
        return sqrt(pow(start.first - end.first, 2) +
                    pow(start.second - end.second, 2));
    }

    void recursive_path(nav_msgs::Path &path, std::pair<double, double> start,
                        std::pair<double, double> end, double epsilon,
                        int &index) {
        if (CalcDistance(start, end) < (epsilon) || path.poses.size() > 200)
            return;

        double start_x = (start.first + end.first) / 2.0;
        double start_y = (start.second + end.second) / 2.0;

        index = find_out(path, start);

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = start_x;
        pose.pose.position.y = start_y;
        path.poses.insert(path.poses.begin() + index, pose);

        recursive_path(path, start, std::pair < double, double > {start_x, start_y},
                       epsilon, index);
        recursive_path(path, std::pair < double, double > {start_x, start_y}, end,
                       epsilon, index);
    }

    int find_out(nav_msgs::Path &path, std::pair<double, double> point) {
        int index = 0;
        for (int i = 0; i < path.poses.size(); i++)
            if (path.poses[i].pose.position.x == point.first &&
                path.poses[i].pose.position.y == point.second)
                return ++i;
        return index;
    }

    bool equal(std::pair<double, double> first,
               std::pair<double, double> second) {
        return (first.first == second.first && first.second == second.second)
               ? true
               : false;
    }

    void diff(float *out, float *p_from, const nav_msgs::Path &path,
              const float dist) {
        // out[0] = (p_from[0] - p_to[0]) * dist + (p_from[0] - p_to[0]);
        // out[1] = (p_from[1] - p_to[1]) * dist + (p_from[1] - p_to[1]);
        // out[0] = p_from[0] - p_to[0];
        // out[1] = p_from[1] - p_to[1];
        // sum_the_output_vector(out, path);
        if (int(out[0] * 100) == 0 && int(out[1] * 100) == 0) {
            float a2 = p_from[2];
            float a1 = out[2];
            float ang1 = a2 - a1;
            float ang2 = ((a2 - a1) + 6.28);
            float ang3 = ((a2 - a1) - 6.28);
            float ang = fabs(ang1) < fabs(ang2) ? ang1 : ang2;
            ang = fabs(ang) < fabs(ang3) ? ang : ang3;
            out[2] = ang;
        }

        float a2 = p_from[2];
        float a1 = out[2];
        float ang1 = a2 - a1;
        float ang2 = ((a2 - a1) + 6.28);
        float ang3 = ((a2 - a1) - 6.28);
        float ang = fabs(ang1) < fabs(ang2) ? ang1 : ang2;
        ang = fabs(ang) < fabs(ang3) ? ang : ang3;
        out[2] = ang;
    }

    float angle_err(float a1, float a2) {
        float ang1 = a2 - a1;
        float ang2 = ((a2 - a1) + 6.28);
        float ang3 = ((a2 - a1) - 6.28);

        float ang = fabs(ang1) < fabs(ang2) ? ang1 : ang2;
        ang = fabs(ang) < fabs(ang3) ? ang : ang3;
        return ang;
    }

    void rotate_vector(float *vect, float angle) {
        float tmp[2];
        tmp[0] = vect[0] * cos(angle) - vect[1] * sin(angle);
        tmp[1] = vect[0] * sin(angle) + vect[1] * cos(angle);
        vect[0] = tmp[0];
        vect[1] = tmp[1];
    }

    void publish_debug(const float vect[3], ros::Publisher &publisher) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = vect[0];
        pose.pose.position.y = vect[1];
        pose.pose.position.z = vect[2];
        publisher.publish(pose);
    }

    void publish_move(const float vect[3], ros::Publisher &publisher) {
        geometry_msgs::Pose2D pose;
        pose.x = vect[0];
        pose.y = vect[1];
        pose.theta = vect[2];
        publisher.publish(pose);
        return;
    }

    float getYawFromQuartenion(const geometry_msgs::Quaternion &qMsg) {
        tf::Quaternion q;
        tf::quaternionMsgToTF(qMsg, q);
        tf::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        return static_cast<float>(y);
    }

    float distance(const float *p1, const float *p2) {
        return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
    }

    float calc_speed(float *current_vect, float *current_pose, float *last_pose,
                     ros::Time &time_current, ros::Time &time_last) {
        float time_betwen = (time_current.sec + time_current.nsec / 1000000000.0) -
                            (time_last.sec + time_last.nsec / 1000000000.0);
        float speed = distance(current_pose, last_pose) / time_betwen;
        current_vect[0] = (current_pose[0] - last_pose[0]) / time_betwen;
        current_vect[1] = (current_pose[1] - last_pose[1]) / time_betwen;
        current_vect[2] = (current_pose[2] - last_pose[2]) / time_betwen;
        return speed;
    }

    bool lineOfSight(const geometry_msgs::Pose &pose1,
                     const float *const target_pose,
                     const cost_map::CostMap costmap,
                     const std::string layer, const int minimal_cost) {
        cost_map::Index p1 = {pose1.position.x, pose1.position.y};
        cost_map::Index p2 = {target_pose[0], target_pose[1]};
        for (cost_map::LineIterator iterator(costmap, p1, p2); !iterator.isPastEnd(); ++iterator) {
            if (costmap.at(layer, *iterator) >= minimal_cost) {
                return false;
            }
        }
        return true;
    }
};

#endif

/*
path struct

last element = START position of path
first element = GOAL position of path

*/
