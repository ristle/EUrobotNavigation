#ifndef main_local
#define main_local

#include <body.hpp>

void local_planner()
{
	// create MotionPlanning object with the node handler
	ros::NodeHandle node;
	std::string frame_id;
	std::string big_robot1;
	std::string big_robot2;
	std::string small_robot1;
	std::string small_robot2;
	double inflation_radius;
	//================== check all parametrs =================
	ROS_ASSERT(node.hasParam("grid_map_server/robot1_topic"));
	ROS_ASSERT(node.hasParam("grid_map_server/robot2_topic"));
	ROS_ASSERT(node.hasParam("grid_map_server/robot3_topic"));
	ROS_ASSERT(node.hasParam("grid_map_server/robot4_topic"));
  	ROS_ASSERT(node.hasParam("global_planner/frame_id"));
	ROS_ASSERT(node.hasParam("local_planner/radius"));
	
	//================== read input params ==============
	node.getParam("grid_map_server/robot1_topic", big_robot1);
	node.getParam("grid_map_server/robot2_topic", big_robot2);
	node.getParam("grid_map_server/robot3_topic", small_robot1);
	node.getParam("grid_map_server/robot4_topic", small_robot2);
	node.getParam("local_planner/radius", inflation_radius);
 	node.getParam("global_planner/frame_id", frame_id);
 

	// Planner p(inflation_radius, frame_id);
	LocalPlanning lp(0.5, frame_id);

	lp.move_pub = node.advertise<std_msgs::Bool>("planner/lp/zero_speed", 1);
	lp.local_path_publisher = node.advertise<nav_msgs::Path>("/planner/lp/path", 1);
	lp.BigEnemyPub = node.advertise<nav_msgs::Path>("planner/lp/prediction/Big", 1);
	lp.LittleEnemyPub = node.advertise<nav_msgs::Path>("planner/lp/prediction/Small", 1);

	ros::Subscriber sub10 = node.subscribe("cost_map_server/cost_map", 1, &LocalPlanning::callbackCostMap, &lp);
	ros::Rate loop_rate(30);

	while (!lp.isCostMapExist())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("[lp] got the map");
	
	ros::Subscriber sub1 = node.subscribe("/planner/gp/path_simple", 1, &LocalPlanning::callbackSimplePath, &lp);
	ros::Subscriber sub2 = node.subscribe("/planner/tr/speed", 1, &LocalPlanning::callbackSpeed, &lp);
	ros::Subscriber sub3 = big_robot1 != "null" ? node.subscribe(big_robot1, 1, &LocalPlanning::callbackBigEnemy, &lp) : ros::Subscriber();
	ros::Subscriber sub4 = big_robot2 !="null" ? node.subscribe(big_robot2, 1, &LocalPlanning::callbackMyBigFriend, &lp) : ros::Subscriber();
	ros::Subscriber sub5 = small_robot2 != "null" ? node.subscribe(small_robot2, 1, &LocalPlanning::callbackLittleEnemy, &lp) : ros::Subscriber();
	ros::Subscriber sub6 = small_robot1 != "null" ? node.subscribe(small_robot1, 1, &LocalPlanning::callbackMyLittleFriend, &lp) : ros::Subscriber();
	while (node.ok())
	{
		loop_rate.sleep();

		lp.UpdatePath();	
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planner");
	ROS_INFO("[lp] It isn't working");
	local_planner();

	return 0;
}
#endif