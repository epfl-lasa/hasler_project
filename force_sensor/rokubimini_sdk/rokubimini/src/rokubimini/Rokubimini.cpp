// std
#include <chrono>

// rokubimini
#include <rokubimini/Rokubimini.hpp>

#include <message_logger/message_logger.hpp>

namespace rokubimini
{
bool Rokubimini::loadRokubiminiSetup(const setup::Rokubimini* setup)
{
  name_ = setup->name_;
  configuration_ = setup->configuration_;
  return true;
}

void Rokubimini::startupWithCommunication()
{
  if (configuration_.hasForceTorqueFilter())
  {
    setForceTorqueFilter(configuration_.getForceTorqueFilter());
  }
  if (configuration_.hasImuAccelerationFilter())
  {
    setAccelerationFilter(configuration_.getImuAccelerationFilter());
  }
  if (configuration_.hasImuAngularRateFilter())
  {
    setAngularRateFilter(configuration_.getImuAngularRateFilter());
  }
  if (configuration_.hasImuAccelerationRange())
  {
    setAccelerationRange(configuration_.getImuAccelerationRange());
  }
  if (configuration_.hasImuAngularRateRange())
  {
    setAngularRateRange(configuration_.getImuAngularRateRange());
  }
  if (configuration_.hasSensorConfiguration())
  {
    setSensorConfiguration(configuration_.getSensorConfiguration());
  }
  if (configuration_.hasForceTorqueOffset())
  {
    setForceTorqueOffset(configuration_.getForceTorqueOffset());
  }

  if (configuration_.getUseCustomCalibration() && configuration_.hasSensorCalibration())
  {
    setSensorCalibration(configuration_.getSensorCalibration());
  }
  doStartupWithCommunication();
}
void Rokubimini::startupWithoutCommunication()
{
}

void Rokubimini::clearGoalStateEnum()
{
  // stateMachine_.clearGoalStateEnum();
}

void Rokubimini::errorCb()
{
  for (const auto& error_cb : errorCbs_)
  {
    error_cb.second(getName());
  }
}

void Rokubimini::errorRecoveredCb()
{
  clearGoalStateEnum();
  for (const auto& error_recovered_cb : errorRecoveredCbs_)
  {
    error_recovered_cb.second(getName());
  }
}

bool Rokubimini::deviceIsInErrorState()
{
  return true;
  // return getStatusword().getStateEnum() == fsm::StateEnum::Error;
}

void Rokubimini::fatalCb()
{
  for (const auto& fatal_cb : fatalCbs_)
  {
    fatal_cb.second(getName());
  }
}

void Rokubimini::fatalRecoveredCb()
{
  clearGoalStateEnum();
  for (const auto& fatal_recovered_cb : fatalRecoveredCbs_)
  {
    fatal_recovered_cb.second(getName());
  }
}

bool Rokubimini::deviceIsInFatalState()
{
  return true;
  // return getStatusword().getStateEnum() == fsm::StateEnum::Fatal;
}

void Rokubimini::deviceDisconnectedCb()
{
  statuswordRequested_ = false;
  clearGoalStateEnum();
  for (const auto& device_disconnected_cb : deviceDisconnectedCbs_)
  {
    device_disconnected_cb.second(getName());
  }

  // Set statusword and reading accordingly.
  //  statusword_.resetData();
  Reading reading;
  const auto& stamp = any_measurements::Time::nowWallClock();
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.getWrench().time_ = stamp;
    reading_.getImu().time_ = stamp;
    reading_.setStatusword(statusword_);
    if (getConfiguration().getSetReadingToNanOnDisconnect())
    {
      reading_.getWrench().wrench_.getForce().x() = NAN_VALUE;
      reading_.getWrench().wrench_.getForce().y() = NAN_VALUE;
      reading_.getWrench().wrench_.getForce().z() = NAN_VALUE;
      reading_.getWrench().wrench_.getTorque().x() = NAN_VALUE;
      reading_.getWrench().wrench_.getTorque().y() = NAN_VALUE;
      reading_.getWrench().wrench_.getTorque().z() = NAN_VALUE;

      reading_.getImu().angularVelocity_.x() = NAN_VALUE;
      reading_.getImu().angularVelocity_.y() = NAN_VALUE;
      reading_.getImu().angularVelocity_.z() = NAN_VALUE;
      reading_.getImu().linearAcceleration_.x() = NAN_VALUE;
      reading_.getImu().linearAcceleration_.y() = NAN_VALUE;
      reading_.getImu().linearAcceleration_.z() = NAN_VALUE;
    }
    reading = reading_;
  }

  // External reading callbacks.
  for (const auto& reading_cb : readingCbs_)
  {
    reading_cb.second(getName(), reading);
  }
}

void Rokubimini::deviceReconnectedCb()
{
  // setGoalStateEnum(getConfiguration().getGoalStateEnumStartup());
  for (const auto& device_reconnected_cb : deviceReconnectedCbs_)
  {
    device_reconnected_cb.second(getName());
  }
}

void Rokubimini::stageCommand(const Command& command)
{
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
  stagedCommand_ = command;
  commandIsStaged_ = true;
}

Reading Rokubimini::getReading() const
{
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  return reading_;
}

void Rokubimini::getReading(Reading& reading) const
{
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  reading = reading_;
}

void Rokubimini::setStatusword(Statusword& statusword)
{
  // If the stamp has not changed, we assume it is again the same statusword.
  if (statusword.getStamp() == statusword_.getStamp())
  {
    return;
  }

  // Check if statusword contains new data.
  if (statusword_.isEmpty() || statusword.getData() != statusword_.getData())
  {
    MELO_DEBUG_STREAM("Received new statusword (" << statusword << ").");
    std::vector<std::string> infos;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
    std::vector<std::string> fatals;
    statusword.getMessagesDiff(statusword_, infos, warnings, errors, fatals);
    for (const std::string& info : infos)
    {
      MELO_INFO_STREAM("[" << name_.c_str() << "] " << info);
    }
    for (const std::string& warning : warnings)
    {
      MELO_WARN_STREAM("[" << name_.c_str() << "] " << warning);
    }
    for (const std::string& error : errors)
    {
      MELO_ERROR_STREAM("[" << name_.c_str() << "] " << error);
    }
    for (const std::string& fatal : fatals)
    {
      MELO_ERROR_STREAM("[" << name_.c_str() << "] " << fatal);
    }
  }

  // Always update statusword to set new time stamp.
  statusword_ = statusword;
}

}  // namespace rokubimini
