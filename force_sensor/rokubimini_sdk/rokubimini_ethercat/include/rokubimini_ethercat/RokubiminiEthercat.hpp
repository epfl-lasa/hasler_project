#pragma once

#include <rokubimini/Rokubimini.hpp>
#include <rokubimini_ethercat/RokubiminiEthercatSlave.hpp>

namespace rokubimini
{
namespace ethercat
{
/**
 * @class RokubiminiEthercat
 *
 * @brief The Rokubimini Ethercat class.
 *
 * Inherits from the Rokubimini class. It's the interface
 * in the BRIDGE pattern used. It provides the API to be called by
 * client code and is used for interfacing with the implementation
 * class called RokubiminiEthercatSlave.
 *
*/

class RokubiminiEthercat : public Rokubimini
{
public:
  /**
   * @fn RokubiminiEthercat()
   *
   * @brief Default constructor.
   *
   * The default constructor of the RokubiminiEthercat class.
  */

  RokubiminiEthercat() = default;
  ~RokubiminiEthercat() override = default;

  /**
   * @fn void startupWithCommunication()
   *
   * @brief Starts up a Rokubimini Ethercat device after communication has been established.
   *
   * This method starts up a Rokubimini Ethercat device after the
   * EthercatBusManager has established communication with the device.
   *
  */

  void doStartupWithCommunication() override;

  /**
   * @fn void updateProcessReading()
   *
   * @brief Updates the \a RokubiminiEthercat object with new measurements.
   *
   * This method updates the internal \a Reading variable of \a
   * RokubiminiEthercat, by getting the new values from its
   * implementation \a RokubiminiEthercatSlave.
  */
  void updateProcessReading() override;

  /**
   * @fn void updateSendStagedCommand()
   *
   * @brief Sends a staged Command to the device.
   *
   * This method sends a staged (from the RokubiminiEthercatBusManager) Command to
   * the device.
   *
  */

  void updateSendStagedCommand() override;

  /**
   * @fn bool deviceIsMissing()
   *
   * @brief Checks if the device is missing.
   *
  */

  bool deviceIsMissing() const override;

  /**
   * @fn void shutdownWithCommunication()
   *
   * @brief Shuts down a Rokubimini Ethercat device before
   * communication has been closed.
   *
   * This method shuts down a Rokubimini Ethercat device before the
   * EthercatBusManager has terminated communication with the device.
   *
  */

  void shutdownWithCommunication() override;

  /**
   * @fn void setSlavePointer(const RokubiminiEthercatSlavePtr &slavePtr)
   *
   * @brief Sets a pointer to the implementation.
   *
   * This method realizes the pimpl paradigm. Through it,
   * the RokubiminiEthercat object holds a pointer to its
   * implementation, a RokubiminiEthercatSlave object.
   *
   * @param slavePtr The pointer to the RokubiminiEthercatSlave
   * implementation.
   *
  */

  void setSlavePointer(const RokubiminiEthercatSlavePtr& slavePtr)
  {
    slavePtr_ = slavePtr;
  }

  /**
   * @fn void setState(const uint16_t state)
   *
   * @brief Sets the desired EtherCAT state machine state of the device in the bus.
   *
   *
   * @param state Desired state.
   *
  */

  void setState(const uint16_t state)
  {
    slavePtr_->setState(state);
  }

  /**
   * @fn bool waitForState(const uint16_t state)
   *
   * @brief Wait for an EtherCAT state machine state to be reached.
   *
   *
   * @param state Desired state.
   * @return True if the state has been reached within the timeout.
  */

  bool waitForState(const uint16_t state)
  {
    return slavePtr_->waitForState(state);
  }

  /**
   * @fn bool getSerialNumber(unsigned int &serialNumber)
   *
   * @brief Gets the serial number of the device.
   *
   *
   * @param serialNumber The serial number to be fetched.
   * @return True if the serial number was successfully fetched.
   *
  */
  // Missing: Methods for calling SDO
  bool getSerialNumber(unsigned int& samplingRate) override;

  /**
   * @fn bool getForceTorqueSamplingRate(int &samplingRate)
   *
   * @brief Gets the force torque sampling rate of the device.
   *
   * @param samplingRate The force torque sampling rate to be
   * fetched.
   * @return True if the force torque sampling rate was
   * successfully fetched.
   *
  */

  bool getForceTorqueSamplingRate(int& samplingRate) override;

  /**
   * @fn bool setForceTorqueFilter(const
   * configuration::ForceTorqueFilter &filter)
   *
   * @brief Sets a force torque filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the force torque filter was
   * successfully set.
   *
  */

  bool setForceTorqueFilter(const configuration::ForceTorqueFilter& filter) override;

  /**
   * @fn bool setAccelerationFilter(const unsigned int filter)
   *
   * @brief Sets an acceleration filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the acceleration torque filter was
   * successfully set.
   *
  */

  bool setAccelerationFilter(const unsigned int filter) override;

  /**
   * @fn bool setAngularRateFilter (const unsigned int filter)
   *
   * @brief Sets an angular rate filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the angular rate filter was
   * successfully set.
   *
  */

  bool setAngularRateFilter(const unsigned int filter) override;

  /**
   * @fn bool setAccelerationRange(const unsigned int range)
   *
   * @brief Sets an acceleration range to the device.
   *
   * @param range The range to be set.
   * @return True if the acceleration range was
   * successfully set.
   *
  */

  bool setAccelerationRange(const unsigned int range) override;

  /**
   * @fn bool setAngularRateRange(const unsigned int range)
   *
   * @brief Sets an angular rate range to the device.
   *
   * @param range The range to be set.
   * @return True if the angular rate range was
   * successfully set.
   *
  */

  bool setAngularRateRange(const unsigned int range) override;

  /**
   * @fn bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1> &forceTorqueOffset)
   *
   * @brief Sets a force torque offset to the device.
   *
   * @param forceTorqueOffset The offset to be set.
   * @return True if the offset was
   * successfully set.
   *
  */

  bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset) override;

  /**
   * @fn bool setSensorConfiguration(const configuration::SensorConfiguration &sensorConfiguration)
   *
   * @brief Sets a sensor configuration to the device.
   *
   * @param sensorConfiguration The configuration to be set.
   * @return True if the configuration was
   * successfully set.
   *
  */

  bool setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration) override;

  /**
   * @fn bool setSensorCalibration(const calibration::SensorCalibration &sensorCalibration)
   *
   * @brief Sets a sensor calibration to the device.
   *
   * @param sensorCalibration The calibration to be set.
   * @return True if the calibration was
   * successfully set.
   *
  */

  bool setSensorCalibration(const calibration::SensorCalibration& sensorCalibration) override;

  /**
   * @fn template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value &value)
   *
   * @brief Sends a read SDO to the Ethercat device.
   *
   * @param index Index of the SDO.
   * @param subindex  Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value Return argument, will contain the value which was read.
   * @return True if the SDO read was sent successfully.
   *
  */

  template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

  /**
   * @fn bool sendSdoReadGeneric(const std::string &indexString, const std::string &subindexString, const std::string
   * &valueTypeString, std::string &valueString)
   *
   * @brief Sends a generic reading SDO to the Ethercat device.
   *
   * @param indexString A string containing the index of the SDO.
   * @param subindexString  A string containing the sub-index of the SDO.
   * @param valueTypeString A string containing the type of the value to read.
   * @param valueString A string containing the value to read.
   * @return True if the SDO read was sent successfully.
   *
  */
  bool sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                          const std::string& valueTypeString, std::string& valueString);

  /**
   * @fn template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value)
   *
   * @brief Sends a write SDO to the Ethercat device.
   *
   * @param index Index of the SDO.
   * @param subindex  Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value Value to write.
   * @return True if the SDO write was sent successfully.
   *
  */
  template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value);

  /**
   * @fn bool sendSdoWriteGeneric(const std::string &indexString, const std::string &subindexString, const std::string
   * &valueTypeString, const std::string &valueString)
   *
   * @brief Sends a generic write SDO to the Ethercat device.
   *
   * @param indexString A string containing the index of the SDO.
   * @param subindexString  A string containing the sub-index of the SDO.
   * @param valueTypeString A string containing the type of the value to write.
   * @param valueString A string containing the value to write.
   * @return True if the SDO write was sent successfully.
   *
  */

  bool sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                           const std::string& valueTypeString, const std::string& valueString);

protected:
  /**
   * @var RokubiminiEthercatSlavePtr slavePtr_
   *
   * @brief The pointer to implementation.
   *
  */

  RokubiminiEthercatSlavePtr slavePtr_{ nullptr };
};

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int8_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int16_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int32_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int64_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint8_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint16_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint32_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint64_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     float& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     double& value);

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int8_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int16_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int32_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int64_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint8_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint16_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint32_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint64_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const float value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const double value);

}  // namespace ethercat
}  // namespace rokubimini