#include <rokubimini/configuration/SensorConfiguration.hpp>

namespace rokubimini
{
namespace configuration
{
SensorConfiguration::SensorConfiguration(const uint8_t calibrationMatrixActive,
                                         const uint8_t temperatureCompensationActive, const uint8_t imuActive,
                                         const uint8_t coordinateSystemConfigurationActive,
                                         const uint8_t inertiaCompensationActive,
                                         const uint8_t orientationEstimationActive)
  : calibrationMatrixActive_(calibrationMatrixActive)
  , temperatureCompensationActive_(temperatureCompensationActive)
  , imuActive_(imuActive)
  , coordinateSystemConfigurationActive_(coordinateSystemConfigurationActive)
  , inertiaCompensationActive_(inertiaCompensationActive)
  , orientationEstimationActive_(orientationEstimationActive)
{
}

void SensorConfiguration::fromFile(const yaml_tools::YamlNode& yamlNode)
{
  calibrationMatrixActive_ =
      static_cast<uint8_t>(yamlNode["sensor_configuration"]["calibration_matrix_active"].as<bool>());
  temperatureCompensationActive_ =
      static_cast<uint8_t>(yamlNode["sensor_configuration"]["temperature_compensation_active"].as<bool>());
  imuActive_ = static_cast<uint8_t>(yamlNode["sensor_configuration"]["imu_active"].as<int>());
  coordinateSystemConfigurationActive_ =
      static_cast<uint8_t>(yamlNode["sensor_configuration"]["coordinate_system_active"].as<bool>());
  inertiaCompensationActive_ =
      static_cast<uint8_t>(yamlNode["sensor_configuration"]["inertia_compensation_active"].as<unsigned int>());
  orientationEstimationActive_ =
      static_cast<uint8_t>(yamlNode["sensor_configuration"]["orientation_estimation_active"].as<unsigned int>());
}

void SensorConfiguration::print() const
{
  MELO_INFO_STREAM("calibrationMatrixActive_: " << static_cast<unsigned int>(calibrationMatrixActive_));
  MELO_INFO_STREAM("temperatureCompensationActive_: " << static_cast<unsigned int>(temperatureCompensationActive_));
  MELO_INFO_STREAM("imuActive_: " << static_cast<unsigned int>(imuActive_));
  MELO_INFO_STREAM(
      "coordinateSystemConfigurationActive_: " << static_cast<unsigned int>(coordinateSystemConfigurationActive_));
  MELO_INFO_STREAM("inertiaCompensationActive_: " << static_cast<unsigned int>(inertiaCompensationActive_));
  MELO_INFO_STREAM("orientationEstimationActive_: " << static_cast<unsigned int>(orientationEstimationActive_));
}

}  // namespace configuration
}  // namespace rokubimini