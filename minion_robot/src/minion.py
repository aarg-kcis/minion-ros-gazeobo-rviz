#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def commands(robName,robID):
	rospy.init_node('NH_'+robName+'_'+robID,anonymous=True)
	pub=rospy.Publisher(robName+'_'+robID+'/diff_drive_controller/cmd_vel', Twist, queue_size=10)
	sub=rospy.Subscriber(robName+'_'+robID+'/diff_drive_controller/odom',Odometry, update_pose)
	robot_command = Twist()
	#rate = rospy.Rate(10) # 10hz
	robot_command.linear.x = 0.2
	robot_command.linear.y = 0.0
	robot_command.linear.z = 0.0
	robot_command.angular.x = 0.0
	robot_command.angular.y = 0.0
	robot_command.angular.z = 0.5
	while not rospy.is_shutdown():
		pub.publish(robot_command)
    	#rate.sleep()

def update_pose(msg):
	print msg



if __name__ == '__main__':
	robName = sys.argv[1]
	robID = sys.argv[2]

	try:
		commands(robName,robID)
	except rospy.ROSInterruptException: pass
