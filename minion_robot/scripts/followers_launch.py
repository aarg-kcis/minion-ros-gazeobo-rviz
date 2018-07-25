#!/usr/bin/env python
import rospy
import subprocess

class FollowerLaunch:
    def __init__(self, id, initialPose, lij, phij):
        self.ID         = id
        self.pose       = initialPose
        self.Lij        = lij
        self.Phij       = phij
        self.launch_followers()

    def launch_followers(self):
        print("Launch Formation for Minion {}".format(self.ID)) 
        pkg_name = 'pibot_gazebo'
        file_name = 'formation.launch'
        child = subprocess.Popen(["roslaunch", pkg_name, file_name, "robotID:={}".format(self.ID),\
                                    "x:={}".format(self.pose[0]), "y:={}".format(self.pose[1]), \
                                    "th:={}".format(self.pose[2]), "lij:={}".format(self.Lij), \
                                    "phij:={}".format(self.Phij)])

if __name__ == '__main__':
    rospy.init_node('Launching_Minions_Formation', anonymous=True)
    MAX_ROBOTS = 3
    locations = [[0, -1, 0], [-1, -1, 0], [-1, 0, 0]]
    lij =  [1, 1.414, 1]
    phij = [-1.57, -2.355, -3.14]
    try:
        for robot in range(MAX_ROBOTS):
            FollowerLaunch(robot+2, locations[robot], lij[robot], phij[robot])
    except rospy.ROSInterruptException:
        print("Closing...")
        pass