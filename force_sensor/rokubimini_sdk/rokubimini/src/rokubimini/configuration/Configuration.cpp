#include <rokubimini/configuration/Configuration.hpp>

namespace rokubimini
{
namespace configuration
{
void Configuration::fromFile(const std::string& path)
{
  // blabla load the shit from file
  // Clear the configuration first.
  *this = Configuration();

  yaml_tools::YamlNode yaml_node = yaml_tools::YamlNode::fromFile(path);
  {
    // lock the mutex_ for accessing all the internal variables.
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (yaml_node.hasKey("max_command_age"))
    {
      setMaxCommandAge(yaml_node["max_command_age"].as<double>());
      hasMaxCommandAge_ = true;
    }
    else
    {
      hasMaxCommandAge_ = false;
    }

    if (yaml_node.hasKey("auto_stage_last_command"))
    {
      setAutoStageLastCommand(yaml_node["auto_stage_last_command"].as<bool>());
      hasAutoStageLastCommand_ = true;
    }
    else
    {
      hasAutoStageLastCommand_ = false;
    }

    if (yaml_node.hasKey("set_reading_to_nan_on_disconnect"))
    {
      setSetReadingToNanOnDisconnect(yaml_node["set_reading_to_nan_on_disconnect"].as<bool>());
      hasSetReadingToNanOnDisconnect_ = true;
    }
    else
    {
      hasSetReadingToNanOnDisconnect_ = false;
    }

    if (yaml_node.hasKey("imu_acceleration_range"))
    {
      setImuAccelerationRange(yaml_node["imu_acceleration_range"].as<unsigned int>());
      hasImuAccelerationRange_ = true;
    }
    else
    {
      hasImuAccelerationRange_ = false;
    }

    if (yaml_node.hasKey("imu_angular_rate_range"))
    {
      setImuAngularRateRange(yaml_node["imu_angular_rate_range"].as<unsigned int>());
      hasImuAngularRateRange_ = true;
    }
    else
    {
      hasImuAngularRateRange_ = false;
    }

    if (yaml_node.hasKey("force_torque_filter"))
    {
      ForceTorqueFilter filter;
      filter.fromFile(yaml_node);
      setForceTorqueFilter(filter);
      hasForceTorqueFilter_ = true;
    }
    else
    {
      hasForceTorqueFilter_ = false;
    }

    if (yaml_node.hasKey("imu_acceleration_filter"))
    {
      setImuAccelerationFilter(yaml_node["imu_acceleration_filter"].as<unsigned int>());
      hasImuAccelerationFilter_ = true;
    }
    else
    {
      hasImuAccelerationFilter_ = false;
    }

    if (yaml_node.hasKey("imu_angular_rate_filter"))
    {
      setImuAngularRateFilter(yaml_node["imu_angular_rate_filter"].as<unsigned int>());
      hasImuAngularRateFilter_ = true;
    }
    else
    {
      hasImuAngularRateFilter_ = false;
    }

    if (yaml_node.hasKey("force_torque_offset"))
    {
      Eigen::Matrix<double, 6, 1> offset;
      offset(0, 0) = yaml_node["force_torque_offset"]["Fx"].as<double>();
      offset(1, 0) = yaml_node["force_torque_offset"]["Fy"].as<double>();
      offset(2, 0) = yaml_node["force_torque_offset"]["Fz"].as<double>();
      offset(3, 0) = yaml_node["force_torque_offset"]["Tx"].as<double>();
      offset(4, 0) = yaml_node["force_torque_offset"]["Ty"].as<double>();
      offset(5, 0) = yaml_node["force_torque_offset"]["Tz"].as<double>();
      setForceTorqueOffset(offset);
      hasForceTorqueOffset_ = true;
    }
    else
    {
      hasForceTorqueOffset_ = false;
    }

    if (yaml_node.hasKey("sensor_configuration"))
    {
      SensorConfiguration configuration;
      configuration.fromFile(yaml_node);
      setSensorConfiguration(configuration);
      hasSensorConfiguration_ = true;
    }
    else
    {
      hasSensorConfiguration_ = false;
    }

    if (yaml_node.hasKey("use_custom_calibration"))
    {
      setUseCustomCalibration(yaml_node["use_custom_calibration"].as<bool>());
      hasUseCustomCalibration_ = true;
    }
    else
    {
      hasUseCustomCalibration_ = false;
    }

    if (yaml_node.hasKey("sensor_calibration"))
    {
      calibration::SensorCalibration calibration;
      calibration.fromFile(yaml_node);
      setSensorCalibration(calibration);
      hasSensorCalibration_ = true;
    }
    else
    {
      hasSensorCalibration_ = false;
    }
  }
}

void Configuration::printConfiguration() const
{
  MELO_INFO_STREAM("maxCommandAge_: " << maxCommandAge_);
  MELO_INFO_STREAM("autoStageLastCommand_: " << autoStageLastCommand_);
  MELO_INFO_STREAM("setReadingToNanOnDisconnect_: " << setReadingToNanOnDisconnect_);
  MELO_INFO_STREAM("useCustomCalibration_: " << useCustomCalibration_);
  MELO_INFO_STREAM("imuAccelerationRange_: " << imuAccelerationRange_);
  MELO_INFO_STREAM("imuAngularRateRange_: " << imuAngularRateRange_);
  MELO_INFO_STREAM("imuAccelerationFilter_: " << imuAccelerationFilter_);
  MELO_INFO_STREAM("imuAngularRateFilter_: " << imuAngularRateFilter_);
  MELO_INFO_STREAM("forceTorqueOffset_:\n" << forceTorqueOffset_);
  forceTorqueFilter_.print();
  sensorConfiguration_.print();
}

Configuration& Configuration::operator=(const Configuration& other)
{
  maxCommandAge_ = other.getMaxCommandAge();
  autoStageLastCommand_ = other.getAutoStageLastCommand();
  setReadingToNanOnDisconnect_ = other.getSetReadingToNanOnDisconnect();
  sensorConfiguration_ = other.getSensorConfiguration();
  forceTorqueFilter_ = other.getForceTorqueFilter();
  forceTorqueOffset_ = other.getForceTorqueOffset();
  useCustomCalibration_ = other.getUseCustomCalibration();
  sensorCalibration_ = other.getSensorCalibration();
  imuAccelerationRange_ = other.imuAccelerationRange_;
  imuAngularRateRange_ = other.imuAngularRateRange_;
  imuAccelerationFilter_ = other.imuAccelerationFilter_;
  imuAngularRateFilter_ = other.imuAngularRateFilter_;
  hasMaxCommandAge_ = other.hasMaxCommandAge();
  hasAutoStageLastCommand_ = other.hasAutoStageLastCommand();
  hasSetReadingToNanOnDisconnect_ = other.hasSetReadingToNanOnDisconnect();
  hasSensorConfiguration_ = other.hasSensorConfiguration();
  hasForceTorqueFilter_ = other.hasForceTorqueFilter();
  hasForceTorqueOffset_ = other.hasForceTorqueOffset();
  hasUseCustomCalibration_ = other.hasUseCustomCalibration();
  hasSensorCalibration_ = other.hasSensorCalibration();
  hasImuAccelerationRange_ = other.hasImuAccelerationRange();
  hasImuAngularRateRange_ = other.hasImuAngularRateRange();
  hasImuAccelerationFilter_ = other.hasImuAccelerationFilter();
  hasImuAngularRateFilter_ = other.hasImuAngularRateFilter();
  return *this;
}

void Configuration::setForceTorqueFilter(const ForceTorqueFilter& forceTorqueFilter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  forceTorqueFilter_ = forceTorqueFilter;
}

const ForceTorqueFilter& Configuration::getForceTorqueFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return forceTorqueFilter_;
}

void Configuration::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  forceTorqueOffset_ = forceTorqueOffset;
}

const Eigen::Matrix<double, 6, 1>& Configuration::getForceTorqueOffset() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return forceTorqueOffset_;
}

void Configuration::setUseCustomCalibration(const bool useCustomCalibration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  useCustomCalibration_ = useCustomCalibration;
}

bool Configuration::getUseCustomCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return useCustomCalibration_;
}

void Configuration::setSetReadingToNanOnDisconnect(const bool setReadingToNanOnDisconnect)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  setReadingToNanOnDisconnect_ = setReadingToNanOnDisconnect;
}

bool Configuration::getSetReadingToNanOnDisconnect() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return setReadingToNanOnDisconnect_;
}

void Configuration::setMaxCommandAge(const double maxCommandAge)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxCommandAge_ = maxCommandAge;
}

double Configuration::getMaxCommandAge() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxCommandAge_;
}

void Configuration::setAutoStageLastCommand(const bool keepSendingLastCommand)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  autoStageLastCommand_ = keepSendingLastCommand;
}

bool Configuration::getAutoStageLastCommand() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return autoStageLastCommand_;
}

void Configuration::setSensorConfiguration(const SensorConfiguration& sensorConfiguration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  sensorConfiguration_ = sensorConfiguration;
}

const SensorConfiguration& Configuration::getSensorConfiguration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return sensorConfiguration_;
}

void Configuration::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  sensorCalibration_ = sensorCalibration;
}

const calibration::SensorCalibration& Configuration::getSensorCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return sensorCalibration_;
}

void Configuration::setImuAccelerationFilter(const unsigned int imuAccelerationFilter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAccelerationFilter_ = imuAccelerationFilter;
}

unsigned int Configuration::getImuAccelerationFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAccelerationFilter_;
}

void Configuration::setImuAngularRateFilter(const unsigned int imuAngularRateFilter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAngularRateFilter_ = imuAngularRateFilter;
}

unsigned int Configuration::getImuAngularRateFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAngularRateFilter_;
}

void Configuration::setImuAccelerationRange(const uint8_t imuAccelerationRange)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAccelerationRange_ = imuAccelerationRange;
}

uint8_t Configuration::getImuAccelerationRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAccelerationRange_;
}

void Configuration::setImuAngularRateRange(const uint8_t imuAngularRateRange)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAngularRateRange_ = imuAngularRateRange;
}

uint8_t Configuration::getImuAngularRateRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAngularRateRange_;
}

bool Configuration::hasImuAngularRateRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAngularRateRange_;
}

bool Configuration::hasMaxCommandAge() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasMaxCommandAge_;
}

bool Configuration::hasAutoStageLastCommand() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasAutoStageLastCommand_;
}

bool Configuration::hasSetReadingToNanOnDisconnect() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasSetReadingToNanOnDisconnect_;
}

bool Configuration::hasSensorConfiguration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasSensorConfiguration_;
}

bool Configuration::hasForceTorqueFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasForceTorqueFilter_;
}

bool Configuration::hasForceTorqueOffset() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasForceTorqueOffset_;
}

bool Configuration::hasUseCustomCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasUseCustomCalibration_;
}

bool Configuration::hasSensorCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasSensorCalibration_;
}

bool Configuration::hasImuAccelerationRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAccelerationRange_;
}

bool Configuration::hasImuAccelerationFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAccelerationFilter_;
}

bool Configuration::hasImuAngularRateFilter() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return hasImuAngularRateFilter_;
}
}  // namespace configuration
}  // namespace rokubimini