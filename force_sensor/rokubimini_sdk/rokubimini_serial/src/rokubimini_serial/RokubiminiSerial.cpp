#include <rokubimini_serial/RokubiminiSerial.hpp>

#include <message_logger/message_logger.hpp>
namespace rokubimini
{
namespace serial
{
void RokubiminiSerial::doStartupWithCommunication()
{
  /*
  ** Print configurations of the sensor
  */
  // configuration_.printConfiguration();
  MELO_DEBUG_STREAM("[" << name_.c_str() << "] Calibration Matrix of the sensor: "
                        << configuration_.getSensorCalibration().getCalibrationMatrix() << std::endl);

  implPtr_->startup();
}

bool RokubiminiSerial::init()
{
  return implPtr_->init();
}

void RokubiminiSerial::updateProcessReading()
{
  /* ATTENTION
  **
  ** All of this is rokubimini_ethercat_sdk code
  **
  */
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    implPtr_->getReading(reading_);

    // Update statusword.
    auto statusword(reading_.getStatusword());
    setStatusword(statusword);
    statuswordRequested_ = false;

    // Handle some errors here

    // Log if activated.
    // if (logger_.logIsActive()) {
    //   logger_.addDataToLog(reading_);
    // }

    // External reading callbacks.
    for (const auto& reading_cb : readingCbs_)
    {
      reading_cb.second(getName(), reading_);
    }
  }

  if (deviceIsMissing())
  {
    Statusword statusword;
    // statusword.setStateEnum(fsm::StateEnum::DeviceMissing);
    setStatusword(statusword);
  }
  else
  {
    // if (statusword_.isEmpty()) {
    //   requestAndSetStatusword();
    // }
    // if (statusword_.isEmpty()) {
    //   return;
    // }
  }

  // const fsm::StateEnum activeState = statusword_.getStateEnum();
  // if (activeState == fsm::StateEnum::NA) {
  //   MELO_WARN_STREAM("The FSM state is not available.");
  //   return;
  // }
  // stateMachine_.updateActiveState(activeState);
}

void RokubiminiSerial::updateSendStagedCommand()
{
  return;
}

void RokubiminiSerial::shutdownWithCommunication()
{
  implPtr_->shutdown();
}

bool RokubiminiSerial::deviceIsMissing() const
{
  return false;
}

bool RokubiminiSerial::getSerialNumber(unsigned int& serialNumber)
{
  return implPtr_->getSerialNumber(serialNumber);
}

bool RokubiminiSerial::getForceTorqueSamplingRate(int& samplingRate)
{
  return implPtr_->getForceTorqueSamplingRate(samplingRate);
}

bool RokubiminiSerial::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  return implPtr_->setForceTorqueFilter(filter);
}

bool RokubiminiSerial::setAccelerationFilter(const unsigned int filter)
{
  return implPtr_->setAccelerationFilter(filter);
}

bool RokubiminiSerial::setAngularRateFilter(const unsigned int filter)
{
  return implPtr_->setAngularRateFilter(filter);
}

bool RokubiminiSerial::setAccelerationRange(const unsigned int range)
{
  return implPtr_->setAccelerationRange(range);
}

bool RokubiminiSerial::setAngularRateRange(const unsigned int range)
{
  return implPtr_->setAngularRateRange(range);
}

bool RokubiminiSerial::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  return implPtr_->setForceTorqueOffset(forceTorqueOffset);
}

bool RokubiminiSerial::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  if (!implPtr_->setSensorConfiguration(sensorConfiguration))
  {
    return false;
  }
  getConfiguration().setSensorConfiguration(sensorConfiguration);
  return true;
}

bool RokubiminiSerial::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  if (!implPtr_->setSensorCalibration(sensorCalibration))
  {
    return false;
  }
  getConfiguration().setSensorCalibration(sensorCalibration);
  return true;
}

bool RokubiminiSerial::setConfigMode()
{
  return implPtr_->setConfigMode();
}

bool RokubiminiSerial::setRunMode()
{
  return implPtr_->setRunMode();
}
bool RokubiminiSerial::setCommunicationSetup(const configuration::SensorConfiguration& sensorConfiguration,
                                             const uint8_t& dataFormat, const uint32_t& baudRate)
{
  return implPtr_->setCommunicationSetup(sensorConfiguration, dataFormat, baudRate);
}

bool RokubiminiSerial::saveConfig()
{
  return implPtr_->saveConfig();
}

bool RokubiminiSerial::loadConfig()
{
  return implPtr_->loadConfig();
}

bool RokubiminiSerial::printUserConfig()
{
  return implPtr_->printUserConfig();
}

bool RokubiminiSerial::setHardwareReset()
{
  return implPtr_->setHardwareReset();
}

bool RokubiminiSerial::setSoftwareReset()
{
  return implPtr_->setSoftwareReset();
}
}  // namespace serial
}  // namespace rokubimini