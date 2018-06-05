#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from turtlesim.msg import Pose
from tf.transformations import quaternion_from_euler

def run():
	rospy.spin()

def update_goal_position(data):
	wayPoints = Path()
	wayPoints_poses = PoseStamped()
	wayPoints.poses = []
	wayPoints.header.frame_id = 'world'
	wayPoints_poses.header.frame_id = 'world'
	wayPoints_poses.pose.position.x = data.x
	wayPoints_poses.pose.position.y = data.y
	q = quaternion_from_euler(0, 0, data.theta)
	wayPoints_poses.pose.orientation.x = q[0];
	wayPoints_poses.pose.orientation.y = q[1];
	wayPoints_poses.pose.orientation.z = q[2];
	wayPoints_poses.pose.orientation.w = q[3];
	wayPoints.poses.append(wayPoints_poses)
	pub.publish(wayPoints)

if __name__ == '__main__':
	robName = sys.argv[1]
	robID = sys.argv[2]
	count = 0;
	goal_subscriber = rospy.Subscriber('/waypoint_minion_'+robID,Pose, update_goal_position)
	rospy.init_node('ppc_test',anonymous=True)
	pub=rospy.Publisher(robName+'_'+robID+'/path_segment', Path, queue_size=1000)
	try:
		run()
	except rospy.ROSInterruptException: pass
