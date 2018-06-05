#!/usr/bin/env python
import rospy
import sys
import tf
import turtlesim.msg
from nav_msgs.msg import Odometry

def update_tf(data):
    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, 0),(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w),
                     rospy.Time.now(), robName+'_'+robID+'_ground_truth',"world")

if __name__ == '__main__':
    robName = sys.argv[1]
    robID = sys.argv[2]
    rospy.init_node('tf_gt_broadcaster')
    pose_subscriber = rospy.Subscriber(robName+'_'+robID+'/ground_truth_pose',Odometry, update_tf)
    rospy.spin()
