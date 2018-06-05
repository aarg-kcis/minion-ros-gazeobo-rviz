#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import numpy as np

class Minion:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('NH_g2g_'+robName+'_'+robID, anonymous=True)

        self.pose = Pose()
        self.ground_truth_pose = Pose()
        self.goal_pose = Pose()
        self.rate = rospy.Rate(10)

        self.goal_subscriber = rospy.Subscriber('/waypoint_minion_'+robID,Pose, self.update_goal_position)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(robName+'_'+robID+'/diff_drive_controller/cmd_vel',Twist, queue_size=100)

        self.odom_publisher = rospy.Publisher(robName+'_'+robID+'/wheel_odometry',Pose, queue_size=100)

        self.pose_subscriber = rospy.Subscriber(robName+'_'+robID+'/ground_truth_pose',Odometry, self.update_ground_truth_pose)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        #self.pose_subscriber = rospy.Subscriber(robName+'_'+robID+'/diff_drive_controller/odom',Odometry, self.update_pose)

        self.listener = tf.TransformListener()



    def update_ground_truth_pose(self,data):
        self.ground_truth_pose.x = data.pose.pose.position.x
        self.ground_truth_pose.y = data.pose.pose.position.y
        quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self.ground_truth_pose.theta = yaw


    def update_pose(self):
        (trans,rot) = self.listener.lookupTransform('/world',robName+'_'+robID+'/base_link',rospy.Time(0))
        self.pose.x = trans[0]
        self.pose.y = trans[1]
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        quaternion = rot
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self.pose.theta = yaw
        odom_msg = Pose()
        odom_msg.x = self.pose.x
        odom_msg.y = self.pose.y
        odom_msg.theta = self.pose.theta
        self.odom_publisher.publish(odom_msg)


    def update_goal_position(self,data):
        self.goal_pose.x = data.x
        self.goal_pose.y = data.y

        # # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.5 #input("Set your tolerance: ")

        vel_msg = Twist()

        if self.euclidean_distance() >= distance_tolerance:

            self.update_pose()
            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel()
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel()


            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)


            # Publish at the desired rate.
            self.rate.sleep()

        else: # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.goal_pose.x - self.ground_truth_pose.x), 2) +
                    pow((self.goal_pose.y - self.ground_truth_pose.y), 2))

    def linear_vel(self,  constant=1.):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        vel = constant * self.euclidean_distance()
        # if vel > 2:
        #     vel = np.sign(vel)*2
        return vel

    def steering_angle(self):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(self.goal_pose.y - self.ground_truth_pose.y, self.goal_pose.x - self.ground_truth_pose.x)

    def angular_vel(self, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        w = constant * (self.steering_angle() - self.ground_truth_pose.theta)
        # if abs(w) > 2:
        #     w = np.sign(w)*2
        return w

    # def move2goal(self):
        # """Moves the turtle to the goal."""
        # goal_pose = Pose()

        # # Get the input from the user.


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    robName = sys.argv[1]
    robID = sys.argv[2]
    try:
        x = Minion()
        x.run()
    except rospy.ROSInterruptException:
        pass
