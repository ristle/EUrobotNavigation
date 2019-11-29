#include <bits/stdc++.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


#include <cost_map_ros/cost_map_ros.hpp>
#include <cost_map_ros/converter.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"

using std_pair = std::pair<double, double>;

typedef struct State {
    double velocity;
    double acceleration;

    State(double v_, double a_) : velocity(v_), acceleration(a_) {}

    State() {
        velocity = 0;
        acceleration = 0;
    }
};

class PlannerTrajectory {
private:
    nav_msgs::Path global_path;
    cost_map::CostMap *costmap_ptr;
    geometry_msgs::PoseStamped Goal_pred;

    std::vector<double> trajectory_x;
    std::vector<double> trajectory_y;

    double average_velocity = 80; // Defualt or get from param //
    double max_velocity = 100;
    double coef_x[6];
    double coef_y[6];
    int frequency = 30; // Defualt or get from param //
    int index;

public:

    PlannerTrajectory(cost_map::CostMap *costmap_, const int &frequency_, const double &max_velocity_,
                      const double &average_velocity_) {
        average_velocity = average_velocity;;
        max_velocity = max_velocity_;
        costmap_ptr = costmap_;
        frequency = frequency_;
        Goal_pred.pose.position.x = 0.0;
        Goal_pred.pose.position.y = 0.0;
    }

    std::tuple<double, double> norm(double iter_x, double iter_y,
                                    double &max_value, const double &value) {
        max_value = !max_value ? 1 : max_value;
        iter_x /= max_value;
        iter_y /= max_value;

        iter_x *= value;
        iter_y *= value;
        return std::make_tuple((iter_x), (iter_y));
    }

    double CalcDifference(geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2) {
        pose1.pose.position.x -= pose2.pose.position.x;
        pose1.pose.position.y -= pose2.pose.position.y;
        return sqrt(pow(pose1.pose.position.x, 2) + pow(pose1.pose.position.y, 2));
    }
    // Построение полинома 5-ой степени чрез матрицы, используя желаемые выходные занчения скорости и ускорения
    // А также времени доезда
    void CalcCoef(double *coef, const double &setpoint, const double &point, const State &Current, const State &Output,
                  double time) {
        coef[0] = setpoint;// Pose X
        coef[1] = time * Current.velocity; // current velocity
        coef[2] = Current.acceleration; // current acceleration
        //////////////////////////////////////////
        // OUTPUT VELOCOTY AND ACCELERATION = {0, 0}
        //////////////////////////////////////////
        // coef[3] = - 3*pow(time,2)*Current.acceleration / 2 - 6*time*Current.velocity + 10 *(point - setpoint);
        // coef[4] = 3*pow(time,2)*Current.acceleration / 2 + 8*time*Current.velocity - 15 * (point - setpoint);
        // coef[5] = -pow(time,2) *Current.acceleration / 2 - 3*time*Current.velocity +6*(point - setpoint);
        double temp = (4 + 3 * pow(time, 2));
        coef[5] = -pow(time, 2) * Output.acceleration / temp + time * Current.acceleration / temp -
                  12 * (point - setpoint) / temp -
                  Current.velocity * time * (6 * time - 12) / temp + 12 * time * Output.velocity / temp;
        coef[4] = time * Output.velocity - Current.velocity * (pow(time, 2) - 3 * time) +
                  time * Current.acceleration / 2 -
                  3 * (point - setpoint) + 3 * coef[5];
        coef[3] = (point - setpoint) - time * Current.velocity - time * Current.acceleration / 2 - coef[4] - coef[5];


        //////////////////////////////////////////
        // OLD VERSION OF MINIMUM JERK выходная скорость - 0.0
        //////////////////////////////////////////
        // double * x = new double;
        // *x =  time;
        // double opr = (36 * pow(*x,11) + 26 * pow(*x,9) - 60 * pow(*x, 10));
        // opr = opr == 0.0 ? 1: opr;
        // coef[3] = (point * (20 * pow(*x, 6)) + Output.velocity * 8 * pow(*x, 7) +
        //     Output.acceleration* pow(*x, 8)) / opr;
        // coef[4] = (- point * (30 * pow(*x, 5)) + Output.velocity* 14 * pow(*x, 6) -
        //     Output.acceleration * 2 * pow(*x, 7)) / opr;
        // coef[5] = (point * (12 * pow(*x, 4)) - Output.velocity * 6 * pow(*x, 5) +
        //     Output.acceleration * pow(*x, 6)) / opr;

        for (int i = 0; i < 6; i++) {
            std::cout << coef[i] << ", ";
        }
        std::cout << std::endl;
    }

    void GetVectorFromTrajectory(nav_msgs::Path &path, std::string &path_layer, State Current[2], State Output[2],
                                 float *move_vect) {
        double average_velocity = 40; // param for the first time
        int c_index = path.poses.size() - 1;
        int g_index = path.poses.size() - 2;
        auto temp_value_x = fabs(-path.poses[g_index].pose.position.x + path.poses[c_index].pose.position.x) * 100;
        auto temp_value_y = fabs(-path.poses[g_index].pose.position.y + path.poses[c_index].pose.position.y) * 100;
        auto temp_sqrt = sqrt(pow(temp_value_x, 2) + pow(temp_value_y, 2));
        double time = temp_sqrt / average_velocity;

        ROS_INFO("[tr] Time is : %f", time);

        ROS_INFO("[tr] Goal is  X: %f \t Y:%f", path.poses[g_index].pose.position.x * 100,
                 path.poses[g_index].pose.position.y * 100);
        ROS_INFO("[tr] Current 1 and 2 deviations: %f \t 2: %f", Current[0].velocity, Current[0].acceleration);

        // для X(t)
        CalcCoef(coef_x, path.poses[c_index].pose.position.x * 100, path.poses[g_index].pose.position.x * 100,
                 Current[0], Output[0], time);
        // для Y(t)
        CalcCoef(coef_y, path.poses[c_index].pose.position.y * 100, path.poses[g_index].pose.position.y * 100,
                 Current[1], Output[1], time);

        double max_value = 0;
        int all_time = time * frequency;

        trajectory_x.clear();
        trajectory_y.clear();

        // выстраивание траектории по производной
        for (int t = 0; t < all_time; t++) {
            double i = t / 100.;
            trajectory_x.push_back(coef_x[1] + 2 * coef_x[2] * i + 3 * coef_x[3] * pow(i, 2) +
                                   4 * coef_x[4] * pow(i, 3) + 5 * coef_x[5] * pow(i, 4));

            trajectory_y.push_back(coef_y[1] + 2 * coef_y[2] * i + 3 * coef_y[3] * pow(i, 2) +
                                   4 * coef_y[4] * pow(i, 3) + 5 * coef_y[5] * pow(i, 4));

            max_value = trajectory_x[t] > max_value ? trajectory_x[t] : max_value;
            max_value = trajectory_y[t] > max_value ? trajectory_y[t] : max_value;
        }
        // TODO implent finding the right index from old trajectory!!!!
        ROS_INFO("[tr] Trajectory Vector is X: %f \t Y: %f", trajectory_x[index], trajectory_x[index]);

        // проверка на аномально малые значения с нормиванием
        trajectory_x[index] = fabs(trajectory_x[index]) > 0.09 ? trajectory_x[index] :
                              trajectory_x[index] > 0 ? 0.01 : -0.01;
        trajectory_y[index] = fabs(trajectory_y[index]) > 0.09 ? trajectory_y[index] :
                              trajectory_x[index] > 0 ? 0.01 : -0.01;

        move_vect[0] = trajectory_x[index];
        move_vect[1] = trajectory_y[index];

        move_vect[0] /= 100;
        move_vect[1] /= 100;

        ROS_INFO("[tr] Output vector is : %f %f", move_vect[0], move_vect[1]);

        index = index < trajectory_x.size() ? index + 1 : trajectory_x.size() - 1;

    }
};