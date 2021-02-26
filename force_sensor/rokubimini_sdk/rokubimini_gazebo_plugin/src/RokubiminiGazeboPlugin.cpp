#include "rokubimini_gazebo_plugin/RokubiminiGazeboPlugin.h"

namespace gazebo
{
RokubiminiGazeboPlugin::RokubiminiGazeboPlugin()
{
}

RokubiminiGazeboPlugin::~RokubiminiGazeboPlugin()
{
  nodeHandle_->shutdown();
  delete nodeHandle_;
}

void RokubiminiGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  robotNameSpace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  pluginTopicName_ = robotNameSpace_ + "/joint_states";
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RokubiminiGazeboPlugin::update, this));
  nodeHandle_ = new ros::NodeHandle("~");

  if (sdf->HasElement("bodyName"))
  {
    jointName_ = sdf->GetElement("bodyName")->GetValue()->GetAsString();
    sensorJoint_ = model_->GetJoint(jointName_);
    if (!sensorJoint_)
      MELO_ERROR_STREAM("[RokubiminiGazeboPlugin] Cannot load joint " << jointName_)
  }
  else
  {
    MELO_ERROR_STREAM("[RokubiminiGazeboPlugin] Cannot find F/T sensor joint")
  }

  if (sdf->HasElement("topicName"))
    readingTopicName_ = sdf->GetElement("topicName")->GetValue()->GetAsString();
  else
    MELO_ERROR_STREAM("[RokubiminiGazeboPlugin] Cannot find topic name")

  if (sdf->HasElement("statePublisherRate"))
    loopUpdateRate_ = 1.0 / sdf->GetElement("statePublisherRate")->Get<double>();
  else
    MELO_ERROR_STREAM("[RokubiminiGazeboPlugin] Cannot find update rate")

  initFilters();
  temperature_ = 45.0;
  initPublishers();

  jointStateRos_.name.resize(1);
  jointStateRos_.position.resize(1);
  jointStateRos_.velocity.resize(1);
  jointStateRos_.effort.resize(1);
  jointStateRos_.name[0] = jointName_;

  jointStateRos_.position[0] = 0.0;
  jointStateRos_.velocity[0] = 0.0;
  jointStateRos_.effort[0] = 0.0;
}

void RokubiminiGazeboPlugin::update()
{
#if GAZEBO_MAJOR_VERSION > 7
  double step = model_->GetWorld()->Physics()->GetMaxStepSize();
  double fraction = fmod(model_->GetWorld()->SimTime().Double() + (step / 2.0), loopUpdateRate_);
#else
  double step = model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
  double fraction = fmod(model_->GetWorld()->GetSimTime().Double() + (step / 2.0), loopUpdateRate_);
#endif
  if (!(fraction >= 0.0 && fraction < step))
    return;

  ros::Time currentTime = ros::Time::now();
  jointStateRos_.header.stamp = currentTime;
  jointStatePublisher_.publish(jointStateRos_);

  auto ft = sensorJoint_->GetForceTorque(0);
  geometry_msgs::WrenchStamped wrench;
  wrench.header.stamp = currentTime;
#if GAZEBO_MAJOR_VERSION > 8
  wrench.wrench.force.x = wrenchFilterFx_.advance(ft.body2Force.X());
  wrench.wrench.force.y = wrenchFilterFy_.advance(ft.body2Force.Y());
  wrench.wrench.force.z = wrenchFilterFz_.advance(ft.body2Force.Z());
  wrench.wrench.torque.x = wrenchFilterMx_.advance(ft.body2Torque.X());
  wrench.wrench.torque.y = wrenchFilterMy_.advance(ft.body2Torque.Y());
  wrench.wrench.torque.z = wrenchFilterMz_.advance(ft.body2Torque.Z());
#else
  wrench.wrench.force.x = wrenchFilterFx_.advance(ft.body2Force.x);
  wrench.wrench.force.y = wrenchFilterFy_.advance(ft.body2Force.y);
  wrench.wrench.force.z = wrenchFilterFz_.advance(ft.body2Force.z);
  wrench.wrench.torque.x = wrenchFilterMx_.advance(ft.body2Torque.x);
  wrench.wrench.torque.y = wrenchFilterMy_.advance(ft.body2Torque.y);
  wrench.wrench.torque.z = wrenchFilterMz_.advance(ft.body2Torque.z);
#endif

  forceTorqueIMUSensorReadingShm ftIMUReading;
  ftIMUReading.getWrench() = any_measurements_ros::fromRos(wrench);
  // Set IMU readings to zero.
  sensor_msgs::Imu imu;
  imu.header.stamp = currentTime;
  ftIMUReading.getImu() = any_measurements_ros::fromRos(imu);
  ftIMUReading.setTemperature(temperature_);
  ftSensorPublisher_->publishAndSend(ftIMUReading, std::chrono::microseconds(200));
}

// Initialise Filters //
void RokubiminiGazeboPlugin::initFilters()
{
  MELO_INFO("[RokubiminiGazeboPlugin::initFilters] Initializing force/torque filters")
  wrenchFilterFx_ = basic_filters::FirstOrderFilterD(loopUpdateRate_, 0.005, 1.0);
  wrenchFilterFy_ = basic_filters::FirstOrderFilterD(loopUpdateRate_, 0.005, 1.0);
  wrenchFilterFz_ = basic_filters::FirstOrderFilterD(loopUpdateRate_, 0.005, 1.0);
  wrenchFilterMx_ = basic_filters::FirstOrderFilterD(loopUpdateRate_, 0.005, 1.0);
  wrenchFilterMy_ = basic_filters::FirstOrderFilterD(loopUpdateRate_, 0.005, 1.0);
  wrenchFilterMz_ = basic_filters::FirstOrderFilterD(loopUpdateRate_, 0.005, 1.0);
}

// Inits the ROS publishers //
void RokubiminiGazeboPlugin::initPublishers()
{
  // Advertise
  jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>(pluginTopicName_, 1);

  // Advertise sensor wrench rostopic with name according to sensor joint name
  auto ftOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(readingTopicName_, *nodeHandle_);
  ftOptions->rosQueueSize_ = 1u;
  ftOptions->rosLatch_ = false;
  ftOptions->autoPublishRos_ = false;
  ftSensorPublisher_ = cosmo_ros::advertiseShmRos<forceTorqueIMUSensorReadingShm, forceTorqueIMUSensorReadingRos,
                                                  FTIMUSensorReadingConversionTrait>(readingTopicName_, ftOptions);
}

GZ_REGISTER_MODEL_PLUGIN(RokubiminiGazeboPlugin)

}  // namespace gazebo
