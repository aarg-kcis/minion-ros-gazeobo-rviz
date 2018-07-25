// #include "piston_plugin.hh"
#ifndef _PISTON_PLUGIN_HH_
#define _PISTON_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#define MAX_FORCE 200

namespace gazebo
{
  /// \brief A plugin to control a piston.
  class piston_plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: piston_plugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, piston plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoint(this->model->GetName() + "/base_piston_moving");
      gzwarn << "Piston joint name : " << this->joint->GetScopedName() << std::endl;

      double error = 0;
      error = this->joint->GetForce(0) - MAX_FORCE ;

      // Setup a P-controller, with a 
      this->pid = common::PID(3, 5, 100);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Default to zero velocity
      double velocity = 0;

      this->SetVelocity(velocity);
      // this->joint->SetVelocityLimit(0, 0.05);
      // std::cout<<s<<std::endl;

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/piston";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &piston_plugin::OnMsg, this);

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));


      ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/piston",1,boost::bind(&piston_plugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread = std::thread(std::bind(&piston_plugin::QueueThread, this));
      
      }

      /// \brief Set the velocity of the Velodyne
      /// \param[in] _vel New target velocity
      public: void SetVelocity(const double &_vel)
      {
        // this->model->GetLink("base_piston_moving")->SetLinearVel({0, 0, _vel});
        return;
        // Set the joint's target velocity.
        this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), _vel);
      }

      /// \brief Handle incoming message
      /// \param[in] _msg Repurpose a vector3 message. This function will
      /// only use the x component.
      private: void OnMsg(ConstVector3dPtr &_msg)
      {
        this->SetVelocity(_msg->x());
      }

      /// \brief Handle an incoming message from ROS
      /// \param[in] _msg A float value that is used to set the velocity
      /// of the Velodyne.
      public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
      {
        gzwarn << "data : " << _msg->data << std::endl;
        // this->model->GetJointController()->SetJointPosition(this->joint->GetScopedName(), _msg->data, 0);
        // return
        this->SetVelocity(_msg->data);
        // double prevForce = this->joint->GetForce(0);
        // double error = prevForce - 1.0;
        // std::map<std::string, double> myMap = this->model->GetJointController()->GetPositions(); 
        // for(std::map<std::string, double >::const_iterator it = myMap.begin();
        //     it != myMap.end(); ++it)
        // {
        //     std::cout << "---" << "\n";
        //     std::cout << it->first << " " << it->second << " " << "\n";
        // }
        // this->joint->SetForce(0, this->pid.Update(error, .1));
        // double prevForce = 0;
        // gzwarn << "error : " << error << std::endl;
        // gzwarn << "update : " << this->pid.Update(error, .1) << std::endl;
        this->model->GetJointController()->Update();
        gzwarn << "force : " << this->joint->GetForce(0) << std::endl;
        // math::Vector3 A = this->joint->GetLinkForce(0);
        // std::cout<<"x: "<< A.x << "y: " << A.y << "z: " << A.z <<std::endl;
        gzwarn << "vel : " << this->joint->GetVelocity(0) << std::endl;
        return;
        if (_msg->data) {
        // flag = 1;
        }
      }

      /// \brief ROS helper function that processes messages
      private: void QueueThread()
      {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
          this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
      }

      /// define a flag to check whether ROS message is recieved or not
      int flag = 0;

      /// \brief A node used for transport
      private: transport::NodePtr node;

      /// \brief A subscriber to a named topic.
      private: transport::SubscriberPtr sub;

      /// \brief Pointer to the model.
      private: physics::ModelPtr model;

      /// \brief Pointer to the joint.
      private: physics::JointPtr joint;

      /// \brief A PID controller for the joint.
      private: common::PID pid;

      /// \brief A node use for ROS transport
      private: std::unique_ptr<ros::NodeHandle> rosNode;

      /// \brief A ROS subscriber
      private: ros::Subscriber rosSub;

      /// \brief A ROS callbackqueue that helps process messages
      private: ros::CallbackQueue rosQueue;

      /// \brief A thread the keeps running the rosQueue
      private: std::thread rosQueueThread;
  };
  GZ_REGISTER_MODEL_PLUGIN(piston_plugin)
}
#endif