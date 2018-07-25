#include "piston_plugin.hh"

using namespace gazebo;

void piston_plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Read the SDF
  if (_sdf->HasElement("piston"))
  {
    for (sdf::ElementPtr elem = _sdf->GetElement("piston"); elem != NULL;
         elem = elem->GetNextElement("piston"))
    {
      // Get piston properties
      PistonProperties properties;
      // piston name is currently an optional property
      if (elem->HasElement("name"))
        properties.name = elem->Get<std::string>("name");

      if (!elem->HasElement("joint"))
      {
        gzwarn << "Invalid SDF: got piston element without joint."
               << std::endl;
        continue;
      }
      std::string jointName = elem->Get<std::string>("joint");
      gzwarn << "Doing Great /......................"
               << std::endl;

      if (_sdf->HasElement("velocity"))
      {
        properties.velocity = elem->Get<double>("velocity");
      }

      if (elem->HasElement("index"))
      {
        properties.jointIndex = elem->Get<unsigned int>("index");
      }
      else
      {
        properties.jointIndex = 0;
      }

      // Store pointer to the joint we will move
      physics::JointPtr joint = _parent->GetJoint(jointName);
      if (!joint)
      {
        gzwarn << "Invalid SDF: piston joint " << jointName << " does not "
               << "exist!" << std::endl;
        continue;
      }
      // joint->SetEffortLimit(properties.jointIndex, properties.maximumTorque);
      this->joints.push_back(joint);
      this->piston.push_back(properties);
    }

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

    // // Set up a physics update callback
    // this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
    //   std::bind(&piston_plugin::WorldUpdateCallback, this)));

    // if (!ros::isInitialized())
    // {
    //   int argc = 0;
    //   char **argv = NULL;
    //   ros::init(argc, argv, "gazebo_client",
    //       ros::init_options::NoSigintHandler);
    // }

    // this->rosNode.reset(new ros::NodeHandle("gazebo_client"));


    // ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/piston",1,boost::bind(&piston_plugin::OnRosMsg, this, _1),
    //     ros::VoidPtr(), &this->rosQueue);

    // this->rosSub = this->rosNode->subscribe(so);

    // this->rosQueueThread = std::thread(std::bind(&piston_plugin::QueueThread, this));
  }
}