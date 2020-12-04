#include <math.h>
#include <rokubimini_msgs/Reading.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <any_measurements/Wrench.hpp>
#include <any_measurements_ros/ConversionTraits.hpp>
#include <any_measurements_ros/ConvertRosMessages.hpp>
#include <basic_filters/filters.hpp>
#include <cosmo_ros/cosmo_ros.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <rokubimini/Reading.hpp>
#include <rokubimini_ros/ConversionTraits.hpp>

#include "message_logger/log/log_messages.hpp"

namespace gazebo
{
class RokubiminiGazeboPlugin : public ModelPlugin
{
public:
  using forceTorqueIMUSensorReadingShm = rokubimini::Reading;
  using forceTorqueIMUSensorReadingRos = rokubimini_msgs::Reading;
  template <typename Msg_, typename MsgRos_>
  using FTIMUSensorReadingConversionTrait = rokubimini_ros::ConversionTraits<Msg_, MsgRos_>;

  RokubiminiGazeboPlugin();

  ~RokubiminiGazeboPlugin() override;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

  void update();

private:
  /// Init joint structures
  void initFilters();
  /// Inits the ROS publishers
  void initPublishers();

  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection_;

  gazebo::physics::JointPtr sensorJoint_;
  std::string jointName_;
  std::string readingTopicName_;

  basic_filters::FirstOrderFilterD wrenchFilterFx_;
  basic_filters::FirstOrderFilterD wrenchFilterFy_;
  basic_filters::FirstOrderFilterD wrenchFilterFz_;
  basic_filters::FirstOrderFilterD wrenchFilterMx_;
  basic_filters::FirstOrderFilterD wrenchFilterMy_;
  basic_filters::FirstOrderFilterD wrenchFilterMz_;
  float temperature_;

  ros::NodeHandle* nodeHandle_ = nullptr;
  ros::Publisher jointStatePublisher_;
  sensor_msgs::JointState jointStateRos_;

  std::string robotNameSpace_;
  std::string pluginTopicName_;

  double loopUpdateRate_;

  // COSMO publisher for simulated F/T sensor
  cosmo_ros::PublisherRosPtr<forceTorqueIMUSensorReadingShm, forceTorqueIMUSensorReadingRos,
                             FTIMUSensorReadingConversionTrait>
      ftSensorPublisher_;
};
}  // namespace gazebo
