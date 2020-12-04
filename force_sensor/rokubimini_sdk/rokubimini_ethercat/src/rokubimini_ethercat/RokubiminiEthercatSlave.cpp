
#include <rokubimini_ethercat/ObjectDictionary.hpp>
#include <rokubimini_ethercat/RokubiminiEthercatSlave.hpp>
#include <rokubimini_ethercat/TxPdoA.hpp>
#include <rokubimini_ethercat/TxPdoB.hpp>
#include <rokubimini_ethercat/TxPdoC.hpp>
#include <rokubimini_ethercat/TxPdoZ.hpp>
#include <rokubimini_ethercat/sdo/RxSdoCalibration.hpp>

namespace rokubimini
{
namespace ethercat
{
RokubiminiEthercatSlave::RokubiminiEthercatSlave(const std::string& name, soem_interface::EthercatBusBase* bus,
                                                 const uint32_t address, const PdoTypeEnum pdoTypeEnum)
  : soem_interface::EthercatSlaveBase(bus, address)
  , name_(name)
  , pdoTypeEnum_(pdoTypeEnum)
  , currentPdoTypeEnum_(PdoTypeEnum::NA)
{
  PdoInfo pdo_info;

  pdo_info.rxPdoId_ = OD_RX_PDO_ID_VAL_A;
  pdo_info.txPdoId_ = OD_TX_PDO_ID_VAL_A;
  pdo_info.rxPdoSize_ = sizeof(RxPdo);
  pdo_info.txPdoSize_ = sizeof(TxPdoA);
  pdo_info.moduleId_ = 0x00119800;
  pdoInfos_.insert({ PdoTypeEnum::A, pdo_info });

  pdo_info.rxPdoId_ = OD_RX_PDO_ID_VAL_B;
  pdo_info.txPdoId_ = OD_TX_PDO_ID_VAL_B;
  pdo_info.rxPdoSize_ = sizeof(RxPdo);
  pdo_info.txPdoSize_ = sizeof(TxPdoB);
  pdo_info.moduleId_ = 0x00219800;
  pdoInfos_.insert({ PdoTypeEnum::B, pdo_info });

  pdo_info.rxPdoId_ = OD_RX_PDO_ID_VAL_C;
  pdo_info.txPdoId_ = OD_TX_PDO_ID_VAL_C;
  pdo_info.rxPdoSize_ = sizeof(RxPdo);
  pdo_info.txPdoSize_ = sizeof(TxPdoC);
  pdo_info.moduleId_ = 0x00319800;
  pdoInfos_.insert({ PdoTypeEnum::C, pdo_info });

  pdo_info.rxPdoId_ = OD_RX_PDO_ID_VAL_A;
  pdo_info.txPdoId_ = OD_TX_PDO_ID_VAL_A;
  pdo_info.rxPdoSize_ = sizeof(RxPdo);
  pdo_info.txPdoSize_ = sizeof(TxPdoZ);
  pdo_info.moduleId_ = 0x00419800;
  pdoInfos_.insert({ PdoTypeEnum::Z, pdo_info });

  pdo_info.rxPdoId_ = OD_RX_PDO_ID_VAL_A;
  pdo_info.txPdoId_ = OD_TX_PDO_ID_VAL_A;
  pdo_info.rxPdoSize_ = sizeof(RxPdo);
  pdo_info.txPdoSize_ = sizeof(TxPdoZ);
  pdo_info.moduleId_ = 0x00519800;
  pdoInfos_.insert({ PdoTypeEnum::EXTIMU, pdo_info });
}

bool RokubiminiEthercatSlave::startup()
{
  // Configure PDO setup
  return configurePdo(pdoTypeEnum_);
}

void RokubiminiEthercatSlave::updateRead()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto& update_stamp = bus_->getUpdateReadStamp();

  switch (getCurrentPdoTypeEnum())
  {
    case PdoTypeEnum::A:
      TxPdoA tx_pdo_a;
      bus_->readTxPdo(address_, tx_pdo_a);
      {
        reading_.getWrench().time_ = update_stamp;
        reading_.getWrench().wrench_.getForce().x() = tx_pdo_a.forceX_;
        reading_.getWrench().wrench_.getForce().y() = tx_pdo_a.forceY_;
        reading_.getWrench().wrench_.getForce().z() = tx_pdo_a.forceZ_;
        reading_.getWrench().wrench_.getTorque().x() = tx_pdo_a.torqueX_;
        reading_.getWrench().wrench_.getTorque().y() = tx_pdo_a.torqueY_;
        reading_.getWrench().wrench_.getTorque().z() = tx_pdo_a.torqueZ_;

        reading_.getImu().time_ = update_stamp;
        reading_.getImu().angularVelocity_.x() = tx_pdo_a.angularRateX_;
        reading_.getImu().angularVelocity_.y() = tx_pdo_a.angularRateY_;
        reading_.getImu().angularVelocity_.z() = tx_pdo_a.angularRateZ_;
        reading_.getImu().linearAcceleration_.x() = tx_pdo_a.accelerationX_;
        reading_.getImu().linearAcceleration_.y() = tx_pdo_a.accelerationY_;
        reading_.getImu().linearAcceleration_.z() = tx_pdo_a.accelerationZ_;
        reading_.getImu().orientation_.x() = tx_pdo_a.estimatedOrientationX_;
        reading_.getImu().orientation_.y() = tx_pdo_a.estimatedOrientationY_;
        reading_.getImu().orientation_.z() = tx_pdo_a.estimatedOrientationZ_;
        reading_.getImu().orientation_.w() = tx_pdo_a.estimatedOrientationW_;

        reading_.setStatusword(Statusword(tx_pdo_a.warningsErrorsFatals_));
        reading_.setTemperature(tx_pdo_a.temperature_);
      }
      break;
    case PdoTypeEnum::B:
      TxPdoB tx_pdo_b;
      bus_->readTxPdo(address_, tx_pdo_b);
      {
        reading_.getWrench().time_ = update_stamp;
        reading_.getWrench().wrench_.getForce().x() = tx_pdo_b.forceX_;
        reading_.getWrench().wrench_.getForce().y() = tx_pdo_b.forceY_;
        reading_.getWrench().wrench_.getForce().z() = tx_pdo_b.forceZ_;
        reading_.getWrench().wrench_.getTorque().x() = tx_pdo_b.torqueX_;
        reading_.getWrench().wrench_.getTorque().y() = tx_pdo_b.torqueY_;
        reading_.getWrench().wrench_.getTorque().z() = tx_pdo_b.torqueZ_;

        reading_.getImu().time_ = update_stamp;
        reading_.getImu().angularVelocity_.x() = tx_pdo_b.angularRateX_;
        reading_.getImu().angularVelocity_.y() = tx_pdo_b.angularRateY_;
        reading_.getImu().angularVelocity_.z() = tx_pdo_b.angularRateZ_;
        reading_.getImu().linearAcceleration_.x() = tx_pdo_b.accelerationX_;
        reading_.getImu().linearAcceleration_.y() = tx_pdo_b.accelerationY_;
        reading_.getImu().linearAcceleration_.z() = tx_pdo_b.accelerationZ_;
        reading_.getImu().orientation_.x() = tx_pdo_b.estimatedOrientationX_;
        reading_.getImu().orientation_.y() = tx_pdo_b.estimatedOrientationY_;
        reading_.getImu().orientation_.z() = tx_pdo_b.estimatedOrientationZ_;
        reading_.getImu().orientation_.w() = tx_pdo_b.estimatedOrientationW_;

        reading_.setStatusword(Statusword(tx_pdo_b.warningsErrorsFatals_));
        reading_.setTemperature(tx_pdo_b.temperature_);
      }

      break;
    case PdoTypeEnum::C:
      TxPdoC tx_pdo_c;
      bus_->readTxPdo(address_, tx_pdo_c);
      {
        reading_.getWrench().time_ = update_stamp;
        reading_.getWrench().wrench_.getForce().x() = tx_pdo_c.forceX_;
        reading_.getWrench().wrench_.getForce().y() = tx_pdo_c.forceY_;
        reading_.getWrench().wrench_.getForce().z() = tx_pdo_c.forceZ_;
        reading_.getWrench().wrench_.getTorque().x() = tx_pdo_c.torqueX_;
        reading_.getWrench().wrench_.getTorque().y() = tx_pdo_c.torqueY_;
        reading_.getWrench().wrench_.getTorque().z() = tx_pdo_c.torqueZ_;

        reading_.getImu().time_ = update_stamp;
        reading_.getImu().angularVelocity_.x() = tx_pdo_c.angularRateX_;
        reading_.getImu().angularVelocity_.y() = tx_pdo_c.angularRateY_;
        reading_.getImu().angularVelocity_.z() = tx_pdo_c.angularRateZ_;
        reading_.getImu().linearAcceleration_.x() = tx_pdo_c.accelerationX_;
        reading_.getImu().linearAcceleration_.y() = tx_pdo_c.accelerationY_;
        reading_.getImu().linearAcceleration_.z() = tx_pdo_c.accelerationZ_;
        reading_.getImu().orientation_.x() = tx_pdo_c.estimatedOrientationX_;
        reading_.getImu().orientation_.y() = tx_pdo_c.estimatedOrientationY_;
        reading_.getImu().orientation_.z() = tx_pdo_c.estimatedOrientationZ_;
        reading_.getImu().orientation_.w() = tx_pdo_c.estimatedOrientationW_;

        reading_.getExternalImu().time_ = update_stamp;
        reading_.getExternalImu().angularVelocity_.x() = tx_pdo_c.externalAngularRateX_;
        reading_.getExternalImu().angularVelocity_.y() = tx_pdo_c.externalAngularRateY_;
        reading_.getExternalImu().angularVelocity_.z() = tx_pdo_c.externalAngularRateZ_;
        reading_.getExternalImu().linearAcceleration_.x() = tx_pdo_c.externalAccelerationX_;
        reading_.getExternalImu().linearAcceleration_.y() = tx_pdo_c.externalAccelerationY_;
        reading_.getExternalImu().linearAcceleration_.z() = tx_pdo_c.externalAccelerationZ_;
        reading_.getExternalImu().orientation_.x() = tx_pdo_c.externalEstimatedOrientationX_;
        reading_.getExternalImu().orientation_.y() = tx_pdo_c.externalEstimatedOrientationY_;
        reading_.getExternalImu().orientation_.z() = tx_pdo_c.externalEstimatedOrientationZ_;
        reading_.getExternalImu().orientation_.w() = tx_pdo_c.externalEstimatedOrientationW_;

        reading_.setStatusword(Statusword(tx_pdo_c.warningsErrorsFatals_));
        reading_.setTemperature(tx_pdo_c.temperature_);
      }

      break;
    case PdoTypeEnum::Z:
    /* dont use break to go to case EXTIMU as it is the same */
    case PdoTypeEnum::EXTIMU:
      TxPdoZ tx_pdo_z;
      bus_->readTxPdo(address_, tx_pdo_z);
      {
        reading_.getWrench().time_ = update_stamp;
        reading_.getWrench().wrench_.getForce().x() = tx_pdo_z.forceX_;
        reading_.getWrench().wrench_.getForce().y() = tx_pdo_z.forceY_;
        reading_.getWrench().wrench_.getForce().z() = tx_pdo_z.forceZ_;
        reading_.getWrench().wrench_.getTorque().x() = tx_pdo_z.torqueX_;
        reading_.getWrench().wrench_.getTorque().y() = tx_pdo_z.torqueY_;
        reading_.getWrench().wrench_.getTorque().z() = tx_pdo_z.torqueZ_;

        reading_.getImu().time_ = update_stamp;
        reading_.getImu().angularVelocity_.x() = tx_pdo_z.angularRateX_;
        reading_.getImu().angularVelocity_.y() = tx_pdo_z.angularRateY_;
        reading_.getImu().angularVelocity_.z() = tx_pdo_z.angularRateZ_;
        reading_.getImu().linearAcceleration_.x() = tx_pdo_z.accelerationX_;
        reading_.getImu().linearAcceleration_.y() = tx_pdo_z.accelerationY_;
        reading_.getImu().linearAcceleration_.z() = tx_pdo_z.accelerationZ_;
        reading_.getImu().orientation_.x() = tx_pdo_z.estimatedOrientationX_;
        reading_.getImu().orientation_.y() = tx_pdo_z.estimatedOrientationY_;
        reading_.getImu().orientation_.z() = tx_pdo_z.estimatedOrientationZ_;
        reading_.getImu().orientation_.w() = tx_pdo_z.estimatedOrientationW_;

        reading_.getExternalImu().time_ = update_stamp;
        reading_.getExternalImu().angularVelocity_.x() = tx_pdo_z.externalAngularRateX_;
        reading_.getExternalImu().angularVelocity_.y() = tx_pdo_z.externalAngularRateY_;
        reading_.getExternalImu().angularVelocity_.z() = tx_pdo_z.externalAngularRateZ_;
        reading_.getExternalImu().linearAcceleration_.x() = tx_pdo_z.externalAccelerationX_;
        reading_.getExternalImu().linearAcceleration_.y() = tx_pdo_z.externalAccelerationY_;
        reading_.getExternalImu().linearAcceleration_.z() = tx_pdo_z.externalAccelerationZ_;
        reading_.getExternalImu().orientation_.x() = tx_pdo_z.externalEstimatedOrientationX_;
        reading_.getExternalImu().orientation_.y() = tx_pdo_z.externalEstimatedOrientationY_;
        reading_.getExternalImu().orientation_.z() = tx_pdo_z.externalEstimatedOrientationZ_;
        reading_.getExternalImu().orientation_.w() = tx_pdo_z.externalEstimatedOrientationW_;

        reading_.setStatusword(Statusword(tx_pdo_z.warningsErrorsFatals_));
        reading_.setTemperature(tx_pdo_z.temperature_);
      }
      break;
    default:
      break;
  }
}

bool RokubiminiEthercatSlave::getSerialNumber(unsigned int& serialNumber)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bool success = true;
  uint32_t serial_number;
  success &= sendSdoRead(OD_IDENTITY_ID, OD_IDENTITY_SID_SERIAL_NUMBER, false, serial_number);
  serialNumber = static_cast<unsigned int>(serial_number);
  MELO_DEBUG("[%s] Reading serial number: %u", name_.c_str(), serialNumber);
  return success;
}

bool RokubiminiEthercatSlave::getForceTorqueSamplingRate(int& samplingRate)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bool success = true;
  int16_t sampling_rate;
  success &= sendSdoRead(OD_SAMPLING_RATE_ID, 0x00, false, sampling_rate);
  samplingRate = static_cast<int>(sampling_rate);
  MELO_DEBUG("[%s] Force/Torque sampling rate: %d Hz", name_.c_str(), samplingRate);
  return success;
}

bool RokubiminiEthercatSlave::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MELO_DEBUG("[%s] Setting force/torque filter", name_.c_str());
  MELO_DEBUG("[%s] \tchop: %u", name_.c_str(), filter.getChopEnable());
  MELO_DEBUG("[%s] \tfast: %u", name_.c_str(), filter.getFastEnable());
  MELO_DEBUG("[%s] \tskip: %u", name_.c_str(), filter.getSkipEnable());
  MELO_DEBUG("[%s] \tsize: %u", name_.c_str(), filter.getSincFilterSize());
  bool success = true;
  /* the following order is mandatory as the sinc filter size range is depending on the other values.
   * Thats why has to be the last one to be changed */
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_CHOP_ENABLE, false, filter.getChopEnable());
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_FAST_ENABLE, false, filter.getFastEnable());
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_FIR_DISABLE, false, filter.getSkipEnable());
  success &=
      sendSdoWrite(OD_FORCE_TORQUE_FILTER_ID, OD_FORCE_TORQUE_FILTER_SID_SINC_SIZE, false, filter.getSincFilterSize());
  return success;
}

bool RokubiminiEthercatSlave::setAccelerationFilter(const unsigned int filter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MELO_DEBUG("[%s] Setting acceleration filter: %u", name_.c_str(), filter);
  bool success = true;
  success &= sendSdoWrite(OD_ACCELERATION_FILTER_ID, 0x00, false, static_cast<uint8_t>(filter));
  return success;
}

bool RokubiminiEthercatSlave::setAngularRateFilter(const unsigned int filter)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MELO_DEBUG("[%s] Setting angular rate filter: %u", name_.c_str(), filter);
  bool success = true;
  success &= sendSdoWrite(OD_ANGULAR_RATE_FILTER_ID, 0x00, false, static_cast<uint8_t>(filter));
  return success;
}

bool RokubiminiEthercatSlave::setAccelerationRange(const unsigned int range)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MELO_DEBUG("[%s] Setting acceleration range: %u", name_.c_str(), range);
  bool success = true;
  success &= sendSdoWrite(OD_ACCELERATION_RANGE_ID, 0x00, false, static_cast<uint8_t>(range));
  return success;
}

bool RokubiminiEthercatSlave::setAngularRateRange(const unsigned int range)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MELO_DEBUG("[%s] Setting angular rate range: %u", name_.c_str(), range);
  bool success = true;
  success &= sendSdoWrite(OD_ANGULAR_RATE_RANGE_ID, 0x00, false, static_cast<uint8_t>(range));
  return success;
}

bool RokubiminiEthercatSlave::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MELO_DEBUG_STREAM("[" << name_.c_str() << "] Setting Force/Torque offset: " << forceTorqueOffset << std::endl);
  bool success = true;
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_1, false,
                          static_cast<float>(forceTorqueOffset(0, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_2, false,
                          static_cast<float>(forceTorqueOffset(1, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_3, false,
                          static_cast<float>(forceTorqueOffset(2, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_4, false,
                          static_cast<float>(forceTorqueOffset(3, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_5, false,
                          static_cast<float>(forceTorqueOffset(4, 0)));
  success &= sendSdoWrite(OD_SENSOR_FORCE_TORQUE_OFFSET_ID, OD_SENSOR_FORCE_TORQUE_OFFSET_SID_6, false,
                          static_cast<float>(forceTorqueOffset(5, 0)));
  return success;
}

bool RokubiminiEthercatSlave::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  MELO_DEBUG("[%s] Setting sensor configuration", name_.c_str());
  bool success = true;

  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_CALIBRATION_MATRIX_ACTIVE, false,
                          sensorConfiguration.getCalibrationMatrixActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_TEMPERATURE_COMPENSATION, false,
                          sensorConfiguration.getTemperatureCompensationActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_IMU_ACTIVE, false,
                          sensorConfiguration.getImuActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_COORD_SYSTEM_CONFIGURATION, false,
                          sensorConfiguration.getCoordinateSystemConfigurationActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_INERTIA_COMPENSATION, false,
                          sensorConfiguration.getInertiaCompensationActive());
  success &= sendSdoWrite(OD_SENSOR_CONFIGURATION_ID, OD_SENSOR_CONFIGURATION_SID_ORIENTATION_ESTIMATION, false,
                          sensorConfiguration.getOrientationEstimationActive());

  return success;
}

bool RokubiminiEthercatSlave::sendCalibrationMatrixEntry(const uint8_t subId, const double entry)
{
  return sendSdoWrite(OD_SENSOR_CALIBRATION_ID, subId, false, static_cast<float>(entry));
}

bool RokubiminiEthercatSlave::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bool success = true;

  success &= sendCalibrationMatrixEntry(0x02, sensorCalibration.getCalibrationMatrix()(0, 0));
  success &= sendCalibrationMatrixEntry(0x03, sensorCalibration.getCalibrationMatrix()(0, 1));
  success &= sendCalibrationMatrixEntry(0x04, sensorCalibration.getCalibrationMatrix()(0, 2));
  success &= sendCalibrationMatrixEntry(0x05, sensorCalibration.getCalibrationMatrix()(0, 3));
  success &= sendCalibrationMatrixEntry(0x06, sensorCalibration.getCalibrationMatrix()(0, 4));
  success &= sendCalibrationMatrixEntry(0x07, sensorCalibration.getCalibrationMatrix()(0, 5));
  success &= sendCalibrationMatrixEntry(0x08, sensorCalibration.getCalibrationMatrix()(1, 0));
  success &= sendCalibrationMatrixEntry(0x09, sensorCalibration.getCalibrationMatrix()(1, 1));
  success &= sendCalibrationMatrixEntry(0x0A, sensorCalibration.getCalibrationMatrix()(1, 2));
  success &= sendCalibrationMatrixEntry(0x0B, sensorCalibration.getCalibrationMatrix()(1, 3));
  success &= sendCalibrationMatrixEntry(0x0C, sensorCalibration.getCalibrationMatrix()(1, 4));
  success &= sendCalibrationMatrixEntry(0x0D, sensorCalibration.getCalibrationMatrix()(1, 5));
  success &= sendCalibrationMatrixEntry(0x0E, sensorCalibration.getCalibrationMatrix()(2, 0));
  success &= sendCalibrationMatrixEntry(0x0F, sensorCalibration.getCalibrationMatrix()(2, 1));
  success &= sendCalibrationMatrixEntry(0x10, sensorCalibration.getCalibrationMatrix()(2, 2));
  success &= sendCalibrationMatrixEntry(0x11, sensorCalibration.getCalibrationMatrix()(2, 3));
  success &= sendCalibrationMatrixEntry(0x12, sensorCalibration.getCalibrationMatrix()(2, 4));
  success &= sendCalibrationMatrixEntry(0x13, sensorCalibration.getCalibrationMatrix()(2, 5));
  success &= sendCalibrationMatrixEntry(0x14, sensorCalibration.getCalibrationMatrix()(3, 0));
  success &= sendCalibrationMatrixEntry(0x15, sensorCalibration.getCalibrationMatrix()(3, 1));
  success &= sendCalibrationMatrixEntry(0x16, sensorCalibration.getCalibrationMatrix()(3, 2));
  success &= sendCalibrationMatrixEntry(0x17, sensorCalibration.getCalibrationMatrix()(3, 3));
  success &= sendCalibrationMatrixEntry(0x18, sensorCalibration.getCalibrationMatrix()(3, 4));
  success &= sendCalibrationMatrixEntry(0x19, sensorCalibration.getCalibrationMatrix()(3, 5));
  success &= sendCalibrationMatrixEntry(0x1A, sensorCalibration.getCalibrationMatrix()(4, 0));
  success &= sendCalibrationMatrixEntry(0x1B, sensorCalibration.getCalibrationMatrix()(4, 1));
  success &= sendCalibrationMatrixEntry(0x1C, sensorCalibration.getCalibrationMatrix()(4, 2));
  success &= sendCalibrationMatrixEntry(0x1D, sensorCalibration.getCalibrationMatrix()(4, 3));
  success &= sendCalibrationMatrixEntry(0x1E, sensorCalibration.getCalibrationMatrix()(4, 4));
  success &= sendCalibrationMatrixEntry(0x1F, sensorCalibration.getCalibrationMatrix()(4, 5));
  success &= sendCalibrationMatrixEntry(0x20, sensorCalibration.getCalibrationMatrix()(5, 0));
  success &= sendCalibrationMatrixEntry(0x21, sensorCalibration.getCalibrationMatrix()(5, 1));
  success &= sendCalibrationMatrixEntry(0x22, sensorCalibration.getCalibrationMatrix()(5, 2));
  success &= sendCalibrationMatrixEntry(0x23, sensorCalibration.getCalibrationMatrix()(5, 3));
  success &= sendCalibrationMatrixEntry(0x24, sensorCalibration.getCalibrationMatrix()(5, 4));
  success &= sendCalibrationMatrixEntry(0x25, sensorCalibration.getCalibrationMatrix()(5, 5));
  success &= sendSdoWrite(OD_SENSOR_CALIBRATION_ID, 0x01, false, sensorCalibration.getPassPhrase());

  return success;
}

void RokubiminiEthercatSlave::setCommand(const Command& command)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  command_ = command;
}

void RokubiminiEthercatSlave::getReading(rokubimini::Reading& reading) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading = reading_;
}

void RokubiminiEthercatSlave::updateWrite()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  RxPdo rx_pdo;
  rx_pdo.digitalOutput_ = static_cast<uint8_t>(0);
  bus_->writeRxPdo(address_, rx_pdo);
}

void RokubiminiEthercatSlave::shutdown()
{
}

void RokubiminiEthercatSlave::setState(const uint16_t state)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  bus_->setState(state, address_);
}

bool RokubiminiEthercatSlave::waitForState(const uint16_t state)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bus_->waitForState(state, address_);
}

RokubiminiEthercatSlave::PdoInfo RokubiminiEthercatSlave::getCurrentPdoInfo() const
{
  // const auto a = pdoInfos_[PdoTypeEnum::A];
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return pdoInfos_.at(currentPdoTypeEnum_);
}

bool RokubiminiEthercatSlave::configurePdo(const PdoTypeEnum pdoTypeEnum)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (pdoTypeEnum == PdoTypeEnum::NA)
  {
    MELO_ERROR("[%s] Invalid EtherCAT PDO Type.", name_.c_str());
    return false;
  }

  // If PDO setup is already active, return.
  if (pdoTypeEnum == getCurrentPdoTypeEnum())
  {
    return true;
  }

  // {
  //   std::lock_guard<std::recursive_mutex> lock(busMutex_);
  //   if (!sendSdoWrite(OD_SWITCH_PDO_ID, OD_SWITCH_PDO_SID, false, pdoInfos_.at(pdoTypeEnum).moduleId_)) {
  //     return false;
  //   }
  // }

  currentPdoTypeEnum_ = pdoTypeEnum;
  return true;
}

}  // namespace ethercat
}  // namespace rokubimini