#include "../include/minion_state_tf_closer.h" 


void tf_closer::storeLatestRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ros::Time actualMessageTime(msg->header.stamp.sec, msg->header.stamp.nsec);

  tfRobWorld.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  tfRobWorld.setRotation(q);

  poseRobWorld_.position.x = msg->pose.position.x;
  poseRobWorld_.position.y = msg->pose.position.y;
  poseRobWorld_.position.z = msg->pose.position.z;
  poseRobWorld_.orientation.w = msg->pose.orientation.w;
  poseRobWorld_.orientation.x = msg->pose.orientation.x;
  poseRobWorld_.orientation.y = msg->pose.orientation.y;
  poseRobWorld_.orientation.z = msg->pose.orientation.z;

  //ROS_INFO("doing some transformation");
  br.sendTransform(tf::StampedTransform(tfRobWorld, actualMessageTime, "world_link", "firefly_"+boost::lexical_cast<string>(robotID)+"/base_link"));



  //Now publish in the MAVOCAP custom message type.
  uavPoseFromGazebo.header = msg->header;
  uavPoseFromGazebo.position.x = msg->pose.position.x;
  uavPoseFromGazebo.position.y = -msg->pose.position.y;
  uavPoseFromGazebo.position.z = -msg->pose.position.z;
  //uavPoseFromGazebo.velocity = ;
  uavPoseFromGazebo.orientation = msg->pose.orientation;
  //uavPoseFromGazebo.covariance = ;
  uav_pose_pub_.publish(uavPoseFromGazebo);

}



int main(int argc, char* argv[])
{



  if (argc < 2)
    {
      ROS_WARN("WARNING: you should specify the robotID \n");
      return 1;
    }
  else
  {
    ros::init(argc, argv, "tf_closer"+boost::lexical_cast<string>(atoi(argv[1])));
    ROS_INFO("Runnin the broadcaster that closes the tf loop for robot number %s",argv[1]);
  }

  ros::NodeHandle nh("~");
  tf_closer node(nh,atoi(argv[1]));

  spin();

  return 0;
}
