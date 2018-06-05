#include <cstdio>
#include <iostream>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <algorithm>
#include "solver_obstaclesV4/MPCsolver.h"    // To import CVXGEN to solve Convex OCP for NOMINAL MPC
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct> // C = kroneckerProduct(A,B);

#include <sys/time.h>

// #include "nmpc_ipopt.hpp"
// #include "IpIpoptApplication.hpp"

// PotentialFunctions
#include <CoTanPotentialFunctions.hpp>

#include <dynamic_reconfigure/server.h>
#include <nmpc_planner/nmpcPlannerParamsConfig.h>



#define GRAVITY 9.8
//PUTH THIS 0 FOR REAL ROBOTS
#define SIM_MODE 1
#define ON_GROUND 0


//#define USE_IPOPT
#undef USE_IPOPT

//#define USE_CVXGEN
#undef USE_CVXGEN

#define USE_CVXGEN_1ROB
//#undef USE_CVXGEN_1ROB
#define PI 3.14159265

#define HUMAN_SPEED 0.8 // m/s

// #define INTERNAL_SUB_STEP 0.01

using namespace std;
using namespace ros;


// using namespace Ipopt;

typedef Eigen::Vector2d Vector2D;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Vector3d Position3D;
typedef Eigen::Vector3d Velocity3D;
typedef Eigen::Vector3d Acceleration3D;
typedef Eigen::Quaterniond Quaternion;

typedef Eigen::Vector3d RotAngle3D;
typedef Eigen::Vector3d AngVelocity3D;
typedef Eigen::Vector3d AngVelocity3Ddot;
typedef Eigen::Matrix3d Matrix3D;
typedef Eigen::Matrix4d Matrix4D;
typedef Eigen::Vector4i Vector4I;
typedef Eigen::Vector4d Vector4D;




class Planner
{

  dynamic_reconfigure::Server<nmpc_planner::nmpcPlannerParamsConfig>  dynamicServer_;

  NodeHandle *nh;

  int selfID_;
  int numRobots_;
  ros::Time currentMeasurementMessageTime;
  ros::Time previousMeasurementMessageTime;

  string robotPoseTopic;
  string robotPoseTopicSuffix;

  string robotPoseTrajTopic;
  string robotPoseTrajTopicSuffix;

  string outputPoseTopic;
  string robotSelfIMUTopic;
  string objectGTTopic;
  string obstacleTopicBase;

  Subscriber subSelfPose_,subSelfIMU_;
  vector<Subscriber> subMatePose_;
  vector<Subscriber> subMatePoseTraj_;
  Subscriber subSelfObstacles_;
  Subscriber subObjectGT_;

  Publisher pubOutPoseSelf_;
  Publisher pubMatlabPoseSelf;
  Publisher pubSelfPoseTraj_;

  nav_msgs::Odometry selfPose;
  geometry_msgs::PoseArray selfPoseTraj;
  geometry_msgs::PoseStamped targetObjectGTPose;
  geometry_msgs::PoseWithCovarianceStamped targetObjectGTVelocity;
  sensor_msgs::Imu selfIMUReading;
  nav_msgs::Odometry matePose;
  bool heardFromMate;

  // make a list of mate poses and pose trajectories. The position of a mate in the list is ID-1 (recall ID has base 0)
  vector<bool> heardFromMates;
  vector<nav_msgs::Odometry> matesPoses;
  vector<geometry_msgs::PoseArray> matesPoseTrajs;
  vector<bool> heardFromMateTrajs;

  turtlesim::Pose outPose;
  geometry_msgs::Pose wayPoint;

  Vector3D ExtForce;

  Eigen::Matrix<double, 3, 16> obstacle_force = Eigen::Matrix<double, 3, 16>::Zero();
  Eigen::Matrix<double, 9, 16> StateInput;

  bool targetEstimationIsActive; // this is a failsafe bool. Until target estimator is active, planner can work on the GT estimate (in real robots this can be set to origin)


  geometry_msgs::PoseStamped outPoseToRviz;
  geometry_msgs::PoseStamped outPoseModifiedToRviz;
  Publisher outPoseRviz_pub, outPoseModifiedRviz_pub;

  std::vector<CoTanRepulsiveGradient*> formationRepulsiveGradientVector;

  std::vector<CoTanRepulsiveGradient*> pointCloudRepulsiveGradientVector;

  geometry_msgs::PoseArray obstaclesFromRobots;


  //planner parameters that need to be read from launch file

  double deltaT;
  double INTERNAL_SUB_STEP;

  bool usingSimulation;
  bool useGTforTarget; // if this is true, NMPC will use GT of the target for feeding into the planner.
  bool useZeroAsFixedTarget; // use this when testing without real target in the real robot setting


  double tFlyToDestinationRadius;
  double tFormationRepulGain;
  double tGoalAttractionGain;
  double neighborDistThreshold;
  double copterDesiredHeightinNED;
  double distanceThresholdToTarget;

  double attractionWeight = 100;

  float total_ang_force;

  double maxObstacleRepulsionForce; // in N

  Eigen::Matrix<double, 9, 16>gradientVector; //constructed as a matrix for convinience.. the rows correspond to x,y,z,vx,vy,vz,ux,uy,uz. last 3 being input accelerations... the 17 = 16(timesteps)+1
  double sumOfGradients;
  double diminishingOmega;

  double x3_prev = -7, y3_prev = -8;

  float current_time = ros::Time::now().toSec();

  public:
    Planner(NodeHandle *_nh, int selfID, int numRobots): nh(_nh), selfID_(selfID), numRobots_(numRobots)
    {
      //ros::Duration(50).sleep();
      obstacle_force = Eigen::Matrix<double, 3, 16>::Zero();
      StateInput = Eigen::Matrix<double, 9, 16>::Zero();
      gradientVector = Eigen::Matrix<double, 9, 16>::Zero();
      sumOfGradients=0;
      total_ang_force = 0;

      ///@hack for now... fix it as per yuyi's plan
      //deltaT = 0.1; for simulation was 0.1 and real robots 0.01
      heardFromMate = false;
      targetEstimationIsActive = false; // target estimation might not have started yet.

      //Initialization time
      currentMeasurementMessageTime = ros::Time::now();
      previousMeasurementMessageTime = currentMeasurementMessageTime;

      nh->getParam("maxObstacleRepulsionForce", maxObstacleRepulsionForce);
      nh->getParam("robotPoseTopic", robotPoseTopic);
      nh->getParam("robotPoseTopicSuffix", robotPoseTopicSuffix);
      nh->getParam("robotPoseTrajTopic", robotPoseTrajTopic);
      nh->getParam("robotPoseTrajTopicSuffix", robotPoseTrajTopicSuffix);
      nh->getParam("outputPoseTopic", outputPoseTopic);
      nh->getParam("robotSelfIMUTopic", robotSelfIMUTopic);
      nh->getParam("objectGTTopic", objectGTTopic);
      nh->getParam("useGTforTarget", useGTforTarget);
      nh->getParam("obstacleTopicBase", obstacleTopicBase);
      nh->getParam("usingSimulation", usingSimulation);
      nh->getParam("useZeroAsFixedTarget", useZeroAsFixedTarget);

      nh->getParam("deltaT", deltaT);
      nh->getParam("INTERNAL_SUB_STEP", INTERNAL_SUB_STEP);

      nh->getParam("tFlyToDestinationRadius", tFlyToDestinationRadius);
      nh->getParam("tFormationRepulGain", tFormationRepulGain);
      nh->getParam("tGoalAttractionGain", tGoalAttractionGain);
      nh->getParam("neighborDistThreshold", neighborDistThreshold);
      nh->getParam("copterDesiredHeightinNED", copterDesiredHeightinNED);
      nh->getParam("distanceThresholdToTarget", distanceThresholdToTarget);

      // Bind dynamic reconfigure callback
      dynamic_reconfigure::Server<nmpc_planner::nmpcPlannerParamsConfig>::CallbackType  callback;
      callback = boost::bind(&Planner::reconf_callback, this, _1);
      dynamicServer_.setCallback(callback);

      subMatePose_.resize(numRobots);
      subMatePoseTraj_.resize(numRobots);
      matesPoses.resize(numRobots);
      matesPoseTrajs.resize(numRobots);
      heardFromMates.resize(numRobots);
      heardFromMateTrajs.resize(numRobots);
      formationRepulsiveGradientVector.resize(numRobots);

      pubOutPoseSelf_ = nh->advertise<turtlesim::Pose>(outputPoseTopic+boost::lexical_cast<string>(selfID_),100);
      //R
      pubMatlabPoseSelf = nh->advertise<geometry_msgs::Pose>("/matlab_Waypoint_"+boost::lexical_cast<string>(selfID_),100);
      //EndR

      pubSelfPoseTraj_ = nh->advertise<geometry_msgs::PoseArray>(robotPoseTrajTopic+boost::lexical_cast<string>(selfID_)+robotPoseTrajTopicSuffix,1000);

      subSelfPose_ = nh->subscribe<nav_msgs::Odometry>(robotPoseTopic+boost::lexical_cast<string>(selfID_)+robotPoseTopicSuffix, 1000, boost::bind(&Planner::selfPoseCallback,this, _1,selfID_));


      subSelfObstacles_ = nh->subscribe<geometry_msgs::PoseArray>(obstacleTopicBase, 1000, boost::bind(&Planner::updateObstaclesCallback,this, _1,selfID_));


      subSelfIMU_ =  nh->subscribe<sensor_msgs::Imu>(robotSelfIMUTopic, 1000, boost::bind(&Planner::selfIMUCallback,this, _1));

      subObjectGT_ = nh->subscribe<geometry_msgs::PoseStamped>(objectGTTopic, 100,boost::bind(&Planner::storeLatestTargetGTPose,this,_1));

      for(int i=1; i<=numRobots; i++)
      {
        //not yet heard from any teammates
        heardFromMates[i-1] = false;
        heardFromMateTrajs[i-1] = false;
    		if(i!=selfID_)
        {
          ROS_INFO("Called pose subscriber for robot %d",i);

          Subscriber subMatePose = nh->subscribe<nav_msgs::Odometry>(robotPoseTopic+boost::lexical_cast<string>(i)+robotPoseTopicSuffix, 100, boost::bind(&Planner::matePoseCallback,this, _1,i));
          subMatePose_[i-1] = subMatePose;

          Subscriber subMatePoseTraj = nh->subscribe<geometry_msgs::PoseArray>(robotPoseTrajTopic+boost::lexical_cast<string>(i)+robotPoseTrajTopicSuffix, 100, boost::bind(&Planner::matePoseTrajectoryCallback,this, _1,i));    	            subMatePoseTraj_[i-1] = subMatePoseTraj;
        }

        formationRepulsiveGradientVector[i-1] = new CoTanRepulsiveGradient("FormationRepulsiveGradient_Neighbor_" + boost::lexical_cast<std::string>(i), neighborDistThreshold, 0.3, 7.0, 5);
      }

      outPoseRviz_pub = nh->advertise<geometry_msgs::PoseStamped>("outPoseRviz_robot"+boost::lexical_cast<string>(selfID_),100);
      outPoseModifiedRviz_pub = nh->advertise<geometry_msgs::PoseStamped>("outPoseModifiedRviz_robot"+boost::lexical_cast<string>(selfID_),100);

      ExtForce = Vector3D::Zero();
      ROS_INFO("Constructor is done");

   }
    void reconf_callback(nmpc_planner::nmpcPlannerParamsConfig&);

    void selfPoseCallback(const nav_msgs::Odometry::ConstPtr&, int);

    void matePoseCallback(const nav_msgs::Odometry::ConstPtr&, int);

    void matePoseTrajectoryCallback(const geometry_msgs::PoseArray::ConstPtr&, int);

    void storeLatestTargetGTPose(const geometry_msgs::PoseStamped::ConstPtr&);

    void updateObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr&, int);

    void selfIMUCallback(const sensor_msgs::Imu::ConstPtr&);

    void avoidTeamMates_byComputingExtForce(double,int);

    double findDistanceDistribution(double, int);

    void repositionDestinationDueToStaticObstacle(float&, float&, float, float, float);

    double quat2eul(nav_msgs::Odometry);

};
