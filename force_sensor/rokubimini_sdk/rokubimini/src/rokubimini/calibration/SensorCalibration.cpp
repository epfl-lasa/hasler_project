#include <rokubimini/calibration/SensorCalibration.hpp>

namespace rokubimini
{
namespace calibration
{
void SensorCalibration::fromFile(const yaml_tools::YamlNode& yamlNode)
{
  // Do something
  calibrationMatrix_(0, 0) = yamlNode["sensor_calibration"]["calibration_matrix"]["1_1"].as<double>();
  calibrationMatrix_(0, 1) = yamlNode["sensor_calibration"]["calibration_matrix"]["1_2"].as<double>();
  calibrationMatrix_(0, 2) = yamlNode["sensor_calibration"]["calibration_matrix"]["1_3"].as<double>();
  calibrationMatrix_(0, 3) = yamlNode["sensor_calibration"]["calibration_matrix"]["1_4"].as<double>();
  calibrationMatrix_(0, 4) = yamlNode["sensor_calibration"]["calibration_matrix"]["1_5"].as<double>();
  calibrationMatrix_(0, 5) = yamlNode["sensor_calibration"]["calibration_matrix"]["1_6"].as<double>();
  calibrationMatrix_(1, 0) = yamlNode["sensor_calibration"]["calibration_matrix"]["2_1"].as<double>();
  calibrationMatrix_(1, 1) = yamlNode["sensor_calibration"]["calibration_matrix"]["2_2"].as<double>();
  calibrationMatrix_(1, 2) = yamlNode["sensor_calibration"]["calibration_matrix"]["2_3"].as<double>();
  calibrationMatrix_(1, 3) = yamlNode["sensor_calibration"]["calibration_matrix"]["2_4"].as<double>();
  calibrationMatrix_(1, 4) = yamlNode["sensor_calibration"]["calibration_matrix"]["2_5"].as<double>();
  calibrationMatrix_(1, 5) = yamlNode["sensor_calibration"]["calibration_matrix"]["2_6"].as<double>();
  calibrationMatrix_(2, 0) = yamlNode["sensor_calibration"]["calibration_matrix"]["3_1"].as<double>();
  calibrationMatrix_(2, 1) = yamlNode["sensor_calibration"]["calibration_matrix"]["3_2"].as<double>();
  calibrationMatrix_(2, 2) = yamlNode["sensor_calibration"]["calibration_matrix"]["3_3"].as<double>();
  calibrationMatrix_(2, 3) = yamlNode["sensor_calibration"]["calibration_matrix"]["3_4"].as<double>();
  calibrationMatrix_(2, 4) = yamlNode["sensor_calibration"]["calibration_matrix"]["3_5"].as<double>();
  calibrationMatrix_(2, 5) = yamlNode["sensor_calibration"]["calibration_matrix"]["3_6"].as<double>();
  calibrationMatrix_(3, 0) = yamlNode["sensor_calibration"]["calibration_matrix"]["4_1"].as<double>();
  calibrationMatrix_(3, 1) = yamlNode["sensor_calibration"]["calibration_matrix"]["4_2"].as<double>();
  calibrationMatrix_(3, 2) = yamlNode["sensor_calibration"]["calibration_matrix"]["4_3"].as<double>();
  calibrationMatrix_(3, 3) = yamlNode["sensor_calibration"]["calibration_matrix"]["4_4"].as<double>();
  calibrationMatrix_(3, 4) = yamlNode["sensor_calibration"]["calibration_matrix"]["4_5"].as<double>();
  calibrationMatrix_(3, 5) = yamlNode["sensor_calibration"]["calibration_matrix"]["4_6"].as<double>();
  calibrationMatrix_(4, 0) = yamlNode["sensor_calibration"]["calibration_matrix"]["5_1"].as<double>();
  calibrationMatrix_(4, 1) = yamlNode["sensor_calibration"]["calibration_matrix"]["5_2"].as<double>();
  calibrationMatrix_(4, 2) = yamlNode["sensor_calibration"]["calibration_matrix"]["5_3"].as<double>();
  calibrationMatrix_(4, 3) = yamlNode["sensor_calibration"]["calibration_matrix"]["5_4"].as<double>();
  calibrationMatrix_(4, 4) = yamlNode["sensor_calibration"]["calibration_matrix"]["5_5"].as<double>();
  calibrationMatrix_(4, 5) = yamlNode["sensor_calibration"]["calibration_matrix"]["5_6"].as<double>();
  calibrationMatrix_(5, 0) = yamlNode["sensor_calibration"]["calibration_matrix"]["6_1"].as<double>();
  calibrationMatrix_(5, 1) = yamlNode["sensor_calibration"]["calibration_matrix"]["6_2"].as<double>();
  calibrationMatrix_(5, 2) = yamlNode["sensor_calibration"]["calibration_matrix"]["6_3"].as<double>();
  calibrationMatrix_(5, 3) = yamlNode["sensor_calibration"]["calibration_matrix"]["6_4"].as<double>();
  calibrationMatrix_(5, 4) = yamlNode["sensor_calibration"]["calibration_matrix"]["6_5"].as<double>();
  calibrationMatrix_(5, 5) = yamlNode["sensor_calibration"]["calibration_matrix"]["6_6"].as<double>();
}
}  // namespace calibration
}  // namespace rokubimini