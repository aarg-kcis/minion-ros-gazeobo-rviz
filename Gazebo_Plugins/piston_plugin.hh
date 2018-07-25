#ifndef GAZEBO_PLUGINS_PISTON_PLUGIN_
#define GAZEBO_PLUGINS_PISTON_PLUGIN_

#include <functional>
#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  // \brief Properties for a model of a piston
  class PistonProperties
  {
    /// \brief An identifier for the piston.
    public: std::string name;

    /// \brief Which joint index is actuated by this piston.
    public: int jointIndex;

    public: double velocity = 0;
  };

  /// \brief Plugin for simulating a piston.
  class GAZEBO_VISIBLE piston_plugin : public ModelPlugin
  {
    /// Documentation inherited
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback on world update event.
    private: void WorldUpdateCallback();

    /// \brief The joints we want to actuate
    private: std::vector<physics::JointPtr> joints;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

      /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    private: physics::JointPtr joint;

    /// \brief Corresponding actuator properties (power, max torque, etc.)
    private: std::vector<PistonProperties> piston;

    /// \brief Connections to events associated with this class.
    private: std::vector<event::ConnectionPtr> connections;

    private: std::unique_ptr<ros::NodeHandle> rosNode;

    private: ros::Subscriber rosSub;

    private: ros::CallbackQueue rosQueue;

    private: std::thread rosQueueThread;

    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
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

  // public: void SetVelocity(const double &_vel)
  // {
  //   // Set the joint's target velocity.
  //   this->model->GetJointController()->SetVelocityTarget(
  //       this->joints->GetScopedName(), _vel);
  // }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(piston_plugin)
}

#endif
