#ifndef GAZEBO_PLUGINS_ACTUATORPLUGIN_
#define GAZEBO_PLUGINS_ACTUATORPLUGIN_

#include <functional>
#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  /// \brief Properties for a model of a rotational actuator
  class ActuatorProperties
  {
    /// \brief An identifier for the actuator.
    public: std::string name;

    /// \brief Which joint index is actuated by this actuator.
    public: int jointIndex;

    /// \brief Mechanical power output of the actuator (Watts)
    public: float power;

    /// \brief Maximum velocity of the actuator (radians per second)
    public: float maximumVelocity;

    /// \brief Maximum torque of the actuator (Newton-meters)
    public: float maximumTorque;

    /// \brief Function used to calculate motor output.
    /// \param[in] float1 Input velocity.
    /// \param[in] float2 Input torque.
    /// \param[in] ActuatorProperties Static properties of this actuator
    /// \return Torque according to the model.
    public: std::function<float (float, float, const ActuatorProperties&)>
              modelFunction;
  };

  /// \brief Plugin for simulating a torque-speed curve for actuators.
  class GAZEBO_VISIBLE ActuatorPlugin : public ModelPlugin
  {
    /// Documentation inherited
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback on world update event.
    private: void WorldUpdateCallback();

    /// \brief The joints we want to actuate
    private: std::vector<physics::JointPtr> joints;

    /// \brief Corresponding actuator properties (power, max torque, etc.)
    private: std::vector<ActuatorProperties> actuators;

    /// \brief Connections to events associated with this class.
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin)
}

#endif
