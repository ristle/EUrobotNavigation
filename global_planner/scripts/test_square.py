#!/usr/bin/env python2
from __future__ import print_function
import roslib
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
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

path = [{
    "x": 1.5
    "y": 1.5,
    # "yaw":  3.14
    "yaw":  0.0
}, {
    "x": 1.0,
    "y": 1.0,
    "yaw":  0.0
}, {
    "x": 2.0,
    "y": 1.0,
    "yaw":  0.0
}, {
    "x": 2.0,
    "y": 2.0,
    "yaw":  0.0
}, {
    "x": 1.0,
    "y": 2.0,
    "yaw":  0.0
}
]

if __name__ == '__main__':
    rospy.init_node('trajectory_sim_node')
    pub = rospy.Publisher('planner/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown(): 
        rospy.loginfo("start test trajectory")
        for point in path:
            rospy.loginfo("go to %f %f %f",point["x"],point["y"],point["yaw"])
            pub.publish(point_to_pose_stamped(point))
            time.sleep(8)
            rate.sleep()

# sudo crxprof -d ./cost_map.calls 12483
# kcachegrind ./cost_map.calls
move_vect: -0.007960 -0.001454 0.005085
move_vect: -0.002034 -0.200000 0.004111
move_vect: -0.002027 -0.200000 0.002861
move_vect: -0.001860 -0.200000 0.002589
move_vect: 0.000238 -0.200000 0.013562
move_vect: 0.008437 -0.200000 0.057972
move_vect: 0.012439 -0.200000 0.076485
move_vect: 0.013679 -0.200000 0.076018
move_vect: 0.010626 -0.200000 0.052734
move_vect: 0.009795 -0.200000 0.045292
move_vect: 0.008520 -0.200000 0.037676
move_vect: 0.008517 -0.200000 0.040388
move_vect: 0.006979 -0.200000 0.034791
move_vect: 0.004701 -0.200000 0.022706
move_vect: 0.004135 -0.200000 0.019092
move_vect: 0.002183 -0.200000 0.009365
move_vect: 0.001532 -0.200000 0.008893
