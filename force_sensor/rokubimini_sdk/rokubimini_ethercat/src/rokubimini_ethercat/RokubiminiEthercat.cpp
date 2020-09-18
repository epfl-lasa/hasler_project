#include <rokubimini_ethercat/RokubiminiEthercat.hpp>

namespace rokubimini
{
namespace ethercat
{
void RokubiminiEthercat::doStartupWithCommunication()
{
}

void RokubiminiEthercat::updateProcessReading()
{
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    slavePtr_->getReading(reading_);

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
    //   statusword.setStateEnum(fsm::StateEnum::DeviceMissing);
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

void RokubiminiEthercat::updateSendStagedCommand()
{
  if (!commandIsStaged_)
  {
    return;
  }

  // Lock the staged command for the entire duration of this function, so that a potential
  // external stage command always overwrites the auto-staged command.
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);

  // Check the age of the command.
  const double command_age = stagedCommand_.getStamp().getElapsedTime().toSeconds();
  const double max_command_age = getConfiguration().getMaxCommandAge();
  if (command_age > max_command_age)
  {
    MELO_ERROR_STREAM("[" << name_.c_str() << "] "
                          << "Command is older than max age (" << command_age << " s > " << max_command_age << " s).");
    return;
  }

  // Send the command.
  slavePtr_->setCommand(stagedCommand_);
  commandIsStaged_ = false;

  // Auto-stage the last command if configured to do so.
  if (getConfiguration().getAutoStageLastCommand())
  {
    Command next_command = stagedCommand_;
    next_command.setStamp(any_measurements::Time::nowWallClock());
    stageCommand(next_command);
  }
}

void RokubiminiEthercat::shutdownWithCommunication()
{
  slavePtr_->shutdown();
}

bool RokubiminiEthercat::deviceIsMissing() const
{
  return false;
}

bool RokubiminiEthercat::getSerialNumber(unsigned int& serialNumber)
{
  return slavePtr_->getSerialNumber(serialNumber);
}

bool RokubiminiEthercat::getForceTorqueSamplingRate(int& samplingRate)
{
  return slavePtr_->getForceTorqueSamplingRate(samplingRate);
}

bool RokubiminiEthercat::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  return slavePtr_->setForceTorqueFilter(filter);
}

bool RokubiminiEthercat::setAccelerationFilter(const unsigned int filter)
{
  return slavePtr_->setAccelerationFilter(filter);
}

bool RokubiminiEthercat::setAngularRateFilter(const unsigned int filter)
{
  return slavePtr_->setAngularRateFilter(filter);
}

bool RokubiminiEthercat::setAccelerationRange(const unsigned int range)
{
  return slavePtr_->setAccelerationRange(range);
}

bool RokubiminiEthercat::setAngularRateRange(const unsigned int range)
{
  return slavePtr_->setAngularRateRange(range);
}

bool RokubiminiEthercat::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  return slavePtr_->setForceTorqueOffset(forceTorqueOffset);
}

bool RokubiminiEthercat::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  if (!slavePtr_->setSensorConfiguration(sensorConfiguration))
  {
    return false;
  }
  getConfiguration().setSensorConfiguration(sensorConfiguration);
  return true;
}

bool RokubiminiEthercat::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  if (!slavePtr_->setSensorCalibration(sensorCalibration))
  {
    return false;
  }
  getConfiguration().setSensorCalibration(sensorCalibration);
  return true;
}

bool RokubiminiEthercat::sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                            const std::string& valueTypeString, std::string& valueString)
{
  return slavePtr_->sendSdoReadGeneric(indexString, subindexString, valueTypeString, valueString);
}

bool RokubiminiEthercat::sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                             const std::string& valueTypeString, const std::string& valueString)
{
  return slavePtr_->sendSdoWriteGeneric(indexString, subindexString, valueTypeString, valueString);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int8_t& value)
{
  return slavePtr_->sendSdoReadInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int16_t& value)
{
  return slavePtr_->sendSdoReadInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int32_t& value)
{
  return slavePtr_->sendSdoReadInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int64_t& value)
{
  return slavePtr_->sendSdoReadInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint8_t& value)
{
  return slavePtr_->sendSdoReadUInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint16_t& value)
{
  return slavePtr_->sendSdoReadUInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint32_t& value)
{
  return slavePtr_->sendSdoReadUInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint64_t& value)
{
  return slavePtr_->sendSdoReadUInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     float& value)
{
  return slavePtr_->sendSdoReadFloat(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     double& value)
{
  return slavePtr_->sendSdoReadDouble(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int8_t value)
{
  return slavePtr_->sendSdoWriteInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int16_t value)
{
  return slavePtr_->sendSdoWriteInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int32_t value)
{
  return slavePtr_->sendSdoWriteInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int64_t value)
{
  return slavePtr_->sendSdoWriteInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint8_t value)
{
  return slavePtr_->sendSdoWriteUInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint16_t value)
{
  return slavePtr_->sendSdoWriteUInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint32_t value)
{
  return slavePtr_->sendSdoWriteUInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint64_t value)
{
  return slavePtr_->sendSdoWriteUInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const float value)
{
  return slavePtr_->sendSdoWriteFloat(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const double value)
{
  return slavePtr_->sendSdoWriteDouble(index, subindex, completeAccess, value);
}

}  // namespace ethercat
}  // namespace rokubimini