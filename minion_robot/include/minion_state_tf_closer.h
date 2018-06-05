
#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sys/time.h>


#include <uav_msgs/uav_pose.h>
#include <turtlesim/Pose.h>

//#define TERMINAL_DEBUG
#undef TERMINAL_DEBUG

//#define DRAW_DEBUG
#undef DRAW_DEBUG

using namespace std;
using namespace ros;

class tf_closer
{

    NodeHandle nh_;
    Publisher minion_pose_pub_;

    //tf broadcaster and transforms
    tf::Transform tfRobWorld;//Object in cam frame,Cam in robot frame, Robot in World frame
    // variables for transforming covariance
    geometry_msgs::Pose poseRobWorld_;

    uav_msgs::uav_pose uavPoseFromGazebo;

    turtlesim::Pose minionPoseFromGazebo;

    int robotID;


  public:
    tf_closer(NodeHandle &_nh, int _robotID): nh_(_nh), robotID(_robotID)
    {
      minion_pose_pub_ = nh_.advertise<turtlesim::Pose>("/pose_minion_"+boost::lexical_cast<string>(robotID),1000);
    }
    void storeLatestRobotPose(const geometry_msgs::PoseStamped::ConstPtr&);
};
