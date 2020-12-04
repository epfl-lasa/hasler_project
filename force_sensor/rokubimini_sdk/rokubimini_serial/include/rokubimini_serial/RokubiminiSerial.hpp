#pragma once

#include <rokubimini/Rokubimini.hpp>
#include <rokubimini_serial/RokubiminiSerialImpl.hpp>

namespace rokubimini
{
namespace serial
{
/**
*@class RokubiminiSerial
*
*@brief The Rokubimini Serial class.
*
*Inherits from the Rokubimini class. It's the interface
*in the BRIDGE pattern used. It provides the API to be called by
*client code and is used for interfacing with the implementation
*class called RokubiminiSerialImpl.
*
*/

class RokubiminiSerial : public Rokubimini
{
public:
  /**
   * @fn RokubiminiSerial()
   *
   * @brief Default constructor.
   *
   * The default constructor of the RokubiminiSerial class.
  */
  RokubiminiSerial() = default;
  ~RokubiminiSerial() override = default;

  /**
   * @fn void doStartupWithCommunication()
   *
   * @brief Starts up a Rokubimini Serial device after communication has been established.
   *
   * This method starts up a Rokubimini Serial device after the
   * SerialBusManager has established communication with the device.
   *
  */
  void doStartupWithCommunication() override;

  /**
   * @fn bool init();
   *
   * @brief Initializes communication with a Rokubimini Serial device.
   *
   * This method is called by the
   * SerialBusManager to establish communication with the device.
   *
  */
  bool init();

  /**
   * @fn void updateProcessReading()
   *
   * @brief Updates the \a RokubiminiSerial object with new measurements.
   *
   * This method updates the internal \a Reading variable of \a
   * RokubiminiSerial, by getting the new values from its
   * implementation \a RokubiminiSerialImpl.
  */
  void updateProcessReading() override;

  /**
   * @fn void updateSendStagedCommand()
   *
   * @brief Sends a staged Command to the device.
   *
   * This method sends a staged (from the BusManager) Command to
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
   * @brief Shuts down a Rokubimini Serial device before
   * communication has been closed.
   *
   * This method shuts down a Rokubimini Serial device before the
   * SerialBusManager has terminated communication with the device.
   *
  */

  void shutdownWithCommunication() override;

  /**
   * @fn void setImplPointer(const RokubiminiSerialImplPtr &implPtr)
   *
   * @brief Sets a pointer to the implementation.
   *
   * This method realizes the pimpl paradigm. Through it,
   * the RokubiminiSerial object holds a pointer to its
   * implementation, a RokubiminiSerialImpl object.
   *
   * @param implPtr The pointer to the RokubiminiSerialImpl
   * implementation.
   *
  */

  void setImplPointer(const RokubiminiSerialImplPtr& implPtr)
  {
    implPtr_ = implPtr;
  }

  /**
   * @fn void setState(const uint16_t state)
   *
   * @brief Sets the state of the device in the bus (unused).
   *
   *
   * @param state Desired state.
   *
  */
  void setState(const uint16_t state)
  {
    implPtr_->setState(state);
  }

  /**
   * @fn bool waitForState(const uint16_t state)
   *
   * @brief Wait for a state to be reached (unused).
   *
   *
   * @param state Desired state.
   * @return True if the state has been reached within the timeout.
  */

  bool waitForState(const uint16_t state)
  {
    return implPtr_->waitForState(state);
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
  bool getSerialNumber(unsigned int& serialNumber) override;

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
   * @fn bool setConfigMode()
   *
   * @brief Sets the device in config mode.
   *
   * @return True if the operation was successful.
  */
  bool setConfigMode();

  /**
   * @fn bool setRunMode()
   *
   * @brief Sets the device in run mode.
   *
   * @return True if the operation was successful.
  */
  bool setRunMode();

  /**
   * @fn bool setHardwareReset()
   *
   * @brief Triggers a hardware reset of the sensor.
   *
   * @return True if the operation was successful.
  */

  bool setHardwareReset();

  /**
   * @fn bool setSoftwareReset()
   *
   * @brief Triggers a software reset of the sensor bringing it to a
   * known state.
   *
   * @return True if the operation was successful.
  */
  bool setSoftwareReset();

  /**
   * @fn bool setCommunicationSetup(const configuration::SensorConfiguration& sensorConfiguration, const uint8_t&
   dataFormat, const uint32_t& baudRate)
   *
   * @brief Sets communication setup for the device. This includes setting the temperature compensation, the matrix
   * calibration, the data format and the baud rate.
   *
   * @param sensorConfiguration The sensor configuration to be set.
   * @param dataFormat The data format (binary = 0, CSV = 1).
   * @param baudRate The desired baud rate (see user manual).
   *
   * @return True if the operation was successful.
  */
  bool setCommunicationSetup(const configuration::SensorConfiguration& sensorConfiguration, const uint8_t& dataFormat,
                             const uint32_t& baudRate);
  /**
   * @fn bool saveConfig()
   *
   * @brief Sets the device in run mode.
   *
   * @return True if the operation was successful.
  */
  bool saveConfig();

  /**
   * @fn bool loadConfig()
   *
   * @brief Loads the configuration of the device.
   *
   * @return True if the operation was successful.
  */
  bool loadConfig();

  /**
   * @fn bool printUserConfig()
   *
   * @brief Prints all the user configurable parameters.
   *
   * @return True if the operation was successful.
  */
  bool printUserConfig();

protected:
  /**
   * @var RokubiminiSerialImplPtr implPtr_
   *
   * @brief The pointer to implementation.
   *
  */

  RokubiminiSerialImplPtr implPtr_{ nullptr };
};

}  // namespace serial
}  // namespace rokubimini