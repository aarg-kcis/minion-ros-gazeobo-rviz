#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist

def commands(robName,robID):
	rospy.init_node('NH_'+robName+'_'+robID,anonymous=True)
	pub=rospy.Publisher(robName+'_'+robID+'_NH_controller/cmd_vel', Twist, queue_size=10)
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




if __name__ == '__main__':
	robName = sys.argv[1]
	robID = sys.argv[2]

	try:
		commands(robName,robID)
	except rospy.ROSInterruptException: pass
