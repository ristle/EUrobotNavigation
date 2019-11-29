#!/usr/bin/env python2
from __future__ import print_function
import roslib
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

# service
import global_planner.srv
# service

import tf_conversions
import time

def point_to_pose_stamped(point):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = point["x"]
    pose.pose.position.y = point["y"]
    yaw = point.get("yaw", 0.0)
    pose.pose.orientation = Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
    return pose
yaw = 1.57/2
path = [{
    "x": 0.5,
    "y": 0.5,
    "yaw":  yaw
}, {
    "x": 1.0,
    "y": 0.5,
    "yaw":  yaw
}, {
    "x": 0.5,
    "y": 0.5,
    "yaw":  yaw
}, {
    "x": 1.0,
    "y": 0.5,
    "yaw":  yaw
}
]
if __name__ == '__main__':
    rospy.init_node('trajectory_sim_node')
    pub = rospy.Publisher('planner/goal', PoseStamped, queue_size=10)
    
    # service
    rospy.wait_for_service('planner/Goal')
    service_goal = rospy.ServiceProxy('planner/Goal',global_planner.srv.Goal)
    rospy.loginfo("Connected to service server stm_driver")
    # service
        
    rate = rospy.Rate(5)
    while not rospy.is_shutdown(): 
        rospy.loginfo("start test trajectory")
        for point in path:
            rospy.loginfo("go to %f %f %f",point["x"],point["y"],point["yaw"])
            # service
            res = service_goal(point_to_pose_stamped(point))
            # service
            
            # pub.publish(point_to_pose_stamped(point))
            time.sleep(8)
            rate.sleep()

# sudo crxprof -d ./cost_map.calls 12483
# kcachegrind ./cost_map.calls
