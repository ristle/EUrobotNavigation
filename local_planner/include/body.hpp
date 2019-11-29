#ifndef BODY__HPP
#define BODY__HPP

#include <planner.hpp>

#define pair_double std::pair<double, double>

// Для хранения положения противника и отслеживания важности данной информации
class MyPoseStamptedArray {
public:
    MyPoseStamptedArray() {}

    MyPoseStamptedArray(geometry_msgs::PoseStamped &pose_, double timer_) {
        pose = pose_;
        timer = timer_;
    }

    double timer;
    geometry_msgs::PoseStamped pose;
};

class LocalPlanning : public Planner {
public:
    LocalPlanning(double inflation_radius_, std::string frame_id_) : Planner(inflation_radius_, frame_id_) {
        inflation_radius = inflation_radius_;
        for (int i = 0; i < 300; i++) {
            for (int j = 0; j < 200; j++)
                our_map[i][j] = 3000.0;
        }
        OurPath->poses.resize(50);

        geometry_msgs::Pose pose;

        pose.position.x = 0.0;
        pose.position.y = 0.0;

        for (int i = 0; i < 50; i++) {
            OurPath->poses[i].pose = pose;
        }

        frame_id = frame_id_;
    }

    virtual ~LocalPlanning() override {
        delete path;
        delete OurPath;
        delete StaticPath;
        delete DynamicPath;
    }

    bool isCostMapExist() {
        return cost_map.getFrameId() == frame_id;
    }

    void callbackBigEnemy(const geometry_msgs::PoseStamped &msg) {
        robots[0].pose = msg;
        robots[0].timer = ros::Time::now().toSec();
    }

    void callbackLittleEnemy(const geometry_msgs::PoseStamped &msg) {
        robots[1].pose = msg;
        robots[1].timer = ros::Time::now().toSec();
    }

    void callbackMyBigFriend(const geometry_msgs::PoseStamped &msg) {
        robots[2].pose = msg;
        robots[2].timer = ros::Time::now().toSec();
    }

    void callbackMyLittleFriend(const geometry_msgs::PoseStamped &msg) {
        robots[3].pose = msg;
        robots[3].timer = ros::Time::now().toSec();
    }

    void callbackSpeed(geometry_msgs::Pose2D speed_) {
        speed = sqrt(pow(speed_.x, 2) + pow(speed_.y, 2));
    }

    void callbackSimplePath(const nav_msgs::Path msg) {
        *path = msg;
    }

    void callbackCostMap(const cost_map_msgs::CostMap::ConstPtr &msg) {
        cost_map::fromMessage(*msg, cost_map);
    }

    virtual void UpdatePath() override {
        int index = 0;
        pair_double rotor;
        // Если путь невозможен, то global_planner отправляет путь с одной точкой, либо точек там нет
        if (path->poses.size() < 2) {
            nav_msgs::Path *_path = new nav_msgs::Path;
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.orientation.w = 1;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            _path->poses.push_back(pose);
            local_path_publisher.publish(*_path);

            delete _path;
            return;
        }
        ROS_INFO("We can build the path! ");
        // Так как массив фиксорованной длины, надо хранить текущий индекс стартовых данных
        add_in_path(OurPath, path->poses[0], OurIndex);

        // Добавления промежуточных точек с расстояние в 2 см - разбиение маршрута для лучшего смещения
        for (int i = 0; i < path->poses.size() - 1; ++i) {
            recursive_path(&(*path), transformPose(path->poses[i]), transformPose(path->poses[i + 1]), 0.02, index);
        }

        bool move = true;
        bool bias = false;
        nav_msgs::Path *output_path = new nav_msgs::Path;

        // Флаг для проверки пробуксовки на карте, когда вьехали в стенку, но продолжаем тыркаться
        // check_zero_speed(OurPath, move, transformPose(OurPath.poses[(OurIndex - 15) % max_size]));

        // Проверка ликвидности пути с прошлой итерации - Прогон по карте на провку наличия препятствий
        check_previous_path(&move, &bias, OurPath, robots);

        // Флаг для остановка - check_zero_speed
        if (move) {
            // Флаг о перестройки старого пути или перестройки из global_planner
            if (bias) {
                int size = path->poses.size();
                // Смещение на край инфлации - безопасной зоны
                path_bias(&(*path), inflation_radius);

                // Смещение точек - смещение точек в сторону вектора, который создается двумя соседними
                // tightening(&(*path));

                // Фильтр  RamerDouglasPeucker
                FilterRosPath(output_path, path, 0.01);
                StaticPath = path;
            } else {
                // Смещение методом Потецниальных полей. Отдельный файл, взятый с прошлой итерации
                get_new_pose(cost_map, StaticPath, DynamicPath, our_map,
                             StaticPath->poses[0].pose.position.x * 100, StaticPath->poses[0].pose.position.y * 100,
                             max_distance);
                // tightening(DynamicPath);

                local_path_publisher.publish(*path);
                // Фильтр  RamerDouglasPeucker
                FilterRosPath(output_path, DynamicPath, 0.01);

                StaticPath = DynamicPath;
            }
        } else {
            nav_msgs::Path *output_path = new nav_msgs::Path;
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.orientation.w = 1;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            output_path->poses.push_back(pose);
            local_path_publisher.publish(*output_path);

            delete output_path;
            return;
        }

        local_path_publisher.publish(*output_path);
    }

private:
    int BigIndex = 0;
    int OurIndex = 0;
    double speed = 0.5;
    int LittleIndex = 0;
    bool make_path = true;
    MyPoseStamptedArray temp;
    double Lower_level = 0.3;
    double Upper_level = 0.35;
    double max_distance = 1.0;
    nav_msgs::Path *StaticPath = new nav_msgs::Path; // Previous path . It's need for changinf using gradient desent.
    cost_map::CostMap cost_map;
    nav_msgs::Path *DynamicPath = new nav_msgs::Path;
    MyPoseStamptedArray robots[4];
    geometry_msgs::PoseStamped pose;


    // Смещение точек. Преобразование из  global_planner
    void path_bias(nav_msgs::Path *path_in, double inflation_radius) {
        pair_double rotor;
        std::vector<int> point_id;
        pair_double bias[3] = {{0.0, 0.0},
                               {0.0, 0.0},
                               {0.0, 0.0}};
        bool near_the_enemy;
        // Проверяем точки по отношению в каждому роботу
        for (int it = 1; it < path_in->poses.size() - 1; it++) {
            auto point_pose = path_in->poses[it].pose.position;
            for (int i = 0; i < 4; i++) {
                near_the_enemy = false;
                auto enemy_pose = robots[i].pose.pose.position;
                if (enemy_pose.x < 0) {
                    continue;
                }
                if (ros::Time::now().toSec() - robots[i].timer > 5) {
                    continue;
                }
                double len_r = CalcDistance(transformPose(path_in->poses[it]), transformPose(robots[i].pose));
                if (len_r > inflation_radius) {
                    continue;
                }
                // Проходит сюда, только если в поле инфляции рядом роботом
                double k = inflation_radius / len_r;
                // Смещение по X и Y отдельно
                double addx = fabs(point_pose.x - enemy_pose.x) * k;
                double addy = fabs(point_pose.y - enemy_pose.y) * k;
                bias[i].first = point_pose.x > enemy_pose.x ? addx : -addx;
                bias[i].second = point_pose.y > enemy_pose.y ? addy : -addy;

                path_in->poses[it].pose.position.x =
                        point_pose.x > enemy_pose.x ? enemy_pose.x + addx : enemy_pose.x - addx;
                path_in->poses[it].pose.position.y =
                        point_pose.y > enemy_pose.y ? enemy_pose.y + addy : enemy_pose.y - addy;
            }
            double time_now = ros::Time::now().toSec();
            if (time_now - robots[0].timer < 5 && time_now - robots[1].timer < 5) {
                // Проверка на двух роботов
                post_checking_point(near_the_enemy, transformPose(robots[it].pose), transformPose(robots[0].pose),
                                    transformPose(robots[1].pose));
            }
            if (near_the_enemy) {
                point_id.push_back(it);
            }
        }
        if (point_id.size() >= 2)
            // Если для двух роботов
            path_in->poses.erase(path_in->poses.begin() + abs(point_id[0]),
                                 path_in->poses.begin() + point_id[point_id.size() - 1]);
        point_id.clear();

        ROS_INFO_THROTTLE(1, " Path Bias Done");
    }

    void post_checking_point(bool &near_the_enemy, pair_double robot_pose, pair_double BigEnemyPose,
                             pair_double LittleEnemyPose) {
        if (CalcDistance(robot_pose, BigEnemyPose) > inflation_radius ||
            CalcDistance(robot_pose, LittleEnemyPose) > inflation_radius) {
            near_the_enemy = true;
            return;
        }
        near_the_enemy = false;
    }

    double CalcDistance(std::pair<double, double> start, std::pair<double, double> end) {
        return sqrt(pow(start.first - end.first, 2) + pow(start.second - end.second, 2));
    }

    pair_double transformPose(geometry_msgs::PoseStamped pose) {
        return pair_double {pose.pose.position.x, pose.pose.position.y};
    }

    void check_previous_path(bool *bias, bool *move, nav_msgs::Path *OurPath, MyPoseStamptedArray *robots) {
        if (*bias == true || OurPath->poses.size() <= 2) {
            *move = true;
            return;
        }
        // Проверка на время и на близость с положениями роботов
        for (int it = 1; it < OurPath->poses.size() - 1; it++) {
            auto point_pose = OurPath->poses[it].pose.position;
            for (int i = 0; i < 4; i++) {
                auto enemy_pose = robots[i].pose.pose.position;
                if (enemy_pose.x < 0) {
                    continue;
                }
                if (ros::Time::now().toSec() - robots[i].timer > 5) {
                    continue;
                }
                double len_r = CalcDistance(transformPose(OurPath->poses[it]), transformPose(robots[i].pose));
                if (len_r > inflation_radius) {
                    continue;
                } else {
                    *move = true;
                    return;
                }
            }
        }
    }

};

/*
			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = 0.0;
			pose.pose.position.y = 0.0;
			output_path.poses.push_back(pose);
*/

#endif
