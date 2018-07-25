#!/usr/bin/env python
import rospy
import subprocess

class PibotAgent:
    def __init__(self, id, initialPose, model=None):
        self.ID         = id
        self.pose       = initialPose
        self.launch_minion()

    def launch_minion(self):
        print("Launch Minion {}".format(self.ID)) 
        pkg_name = 'minion_robot'
        file_name = 'minion_with_ID.launch'
        child = subprocess.Popen(["roslaunch", pkg_name, file_name, "robotID:={}".format(self.ID),\
                                    "x:={}".format(self.pose[0]), "y:={}".format(self.pose[1]), \
                                    "th:={}".format(self.pose[2])])

if __name__ == '__main__':
    rospy.init_node('Launching_Minions', anonymous=True)
    MAX_ROBOTS = 4
    locations = [[0, 0, 0], [0, -1, 0], [-1, -1, 0], [-1, 0, 0]]
    try:
        for robot in range(MAX_ROBOTS):
            PibotAgent(robot+1, locations[robot])
    except rospy.ROSInterruptException:
        print("Closing...")
        pass