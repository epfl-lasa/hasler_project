/*
* RokubiminiSerialImpl.hpp
*  Created on: April, 2020
*  Author(s):  Mike Karamousadakis, Ilias Patsiaouras
*
*  and is based on:
*
* ForceTorqueSensor.hpp
*  Created on:   Dec 19, 2016
*  Author(s):    Christian Gehring, Vassilios Tsounis, Klajd Lika
*/

#pragma once

//! System dependencies
#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <boost/thread.hpp>
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
#include <string>

//! Defines geometric/physical data types
#include <kindr/Core>

//! Device driver specific dependencies
// #include "rokubimini_serial/RokubiminiSerialImplOptions.hpp"
#include <rokubimini_serial/states.hpp>
#include <rokubimini_serial/types.hpp>

#include <rokubimini/Command.hpp>
#include <rokubimini/Reading.hpp>
#include <rokubimini/configuration/Configuration.hpp>
#include <rokubimini/calibration/SensorCalibration.hpp>
#include <rokubimini/configuration/SensorConfiguration.hpp>
#include <rokubimini/configuration/ForceTorqueFilter.hpp>

namespace rokubimini
{
namespace serial
{
/**
 * @union DataStatus
 *
 * @brief The status of the sensor data.
 *
 *
*/

union DataStatus
{
  struct __attribute__((__packed__))
  {
    uint16_t app_took_too_long : 1;
    uint16_t overrange : 1;
    uint16_t invalid_measurements : 1;  // saturation, short, open circuit
    uint16_t raw_measurements : 1;      // gages instead of forces
    uint16_t : 12;                      // reserved
  };
  uint16_t byte;
};

/**
 * @union AppOutput
 *
 * @brief The main output from the sensors in the device.
 *
 *
*/

union AppOutput
{
  struct __attribute__((__packed__))
  {
    DataStatus status;
    float forces[6];
    uint32_t timestamp;
    float temperature;
    // float volts;
  };
  uint8_t bytes[1];
};

/**
 * @union RxFrame
 *
 * @brief The frame transmitted and received via the serial bus.
 *
 *
*/

union RxFrame
{
  struct __attribute__((__packed__))
  {
    uint8_t header;
    AppOutput data;
    uint16_t crc16_ccitt;
  };
  uint8_t bytes[1];
};

/**
*@class RokubiminiSerialImpl
*
*@brief The Rokubimini Serial Implementation class.
*
*It's the implementation in the BRIDGE pattern used. It provides
*the implementation to be called by the interface
*(RokubiminiSerial) in order to communicate with the Serial Device.
*
*/

class RokubiminiSerialImpl
{
public:
  /**
   * @fn RokubiminiSerialImpl()
   *
   * @brief Default constructor is deleted.
   *
   *
  */

  RokubiminiSerialImpl() = delete;

  /**
   * @fn RokubiminiSerialImpl(const std::string &name, const std::string &port, const std::uint32_t &baudRate)
   *
   * @brief Custom constructor for the RokubiminiSerialImpl class.
   *
   * @param name The name of the device.
   * @param port The port to connect to.
   * @param baudRate The baud rate of the serial communication.
   *
  */

  RokubiminiSerialImpl(const std::string& name, const std::string& port, const std::uint32_t& baudRate);

  ~RokubiminiSerialImpl() = default;

  /**
   * @fn bool init()
   *
   * @brief This method initializes internal variables and the device for communication. It connects to the serial-port.
   *
   * @return True if the initialization was successful.
   *
  */

  bool init();

  /**
   * @fn bool startup()
   *
   * @brief This method starts up communication with the device.
   *
   * @return True if the startup was
   * successful.
   *
  */

  bool startup();

  /**
   * @fn void updateRead()
   *
   * @brief This method is called by the BusManager. Each device attached to this bus reads its data from the buffer
   * (not used).
   *
   *
  */

  void updateRead(){};

  /**
   * @fn void updateWrite()
   *
   * @brief This method is called by the BusManager. Each device attached to the bus writes its data to the buffer (not
   * used).
   *
   *
  */

  void updateWrite(){};

  /**
   * @fn void shutdown()
   *
   * @brief Shuts down the device. It automatically shuts-down
   * threads and disconnects from serial-port.
   *
   *
  */

  void shutdown();

  /**
   * @fn void setState(const uint16_t state)
   *
   * @brief Sets the state of the device in the bus (unused).
   *
   * @param state Desired state.
   * @return True if the angular rate filter was
   * successfully set.
   *
  */

  void setState(const uint16_t state){};

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
    return false;
  }

  /**
   * @fn std::string getName() const
   *
   * @brief Accessor for device name.
   *
   * @return The name of the device.
   *
  */

  std::string getName() const
  {
    return name_;
  }

  // Methods for all the SDOs

  /**
   * @fn bool getSerialNumber(unsigned int &serialNumber)
   *
   * @brief Accessor for device serial number.
   *
   * @param serialNumber The serial number to get.
   * @return True if the serial number could be
   * successfully fetched.
   *
  */

  bool getSerialNumber(unsigned int& serialNumber)
  {
    return false;
  }

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

  bool getForceTorqueSamplingRate(int& samplingRate)
  {
    return false;
  }

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

  bool setForceTorqueFilter(const configuration::ForceTorqueFilter& filter);

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

  bool setAccelerationFilter(const unsigned int filter)
  {
    return false;
  }

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

  bool setAngularRateFilter(const unsigned int filter)
  {
    return false;
  }

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

  bool setAccelerationRange(const unsigned int range)
  {
    return false;
  }

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

  bool setAngularRateRange(const unsigned int range)
  {
    return false;
  }

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

  bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset);

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

  bool setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
  {
    return false;
  }

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

  bool setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
  {
    return false;
  }

  /**
   * @fn void setCommand(const Command &command)
   *
   * @brief Sets the internal command variable to @param command.
   *
   * @param command The command to be set.
   *
  */

  void setCommand(const Command& command){};

  /**
   * @fn void getReading(rokubimini::Reading &reading)
   *
   * @brief Gets the internal reading variable.
   *
   * @param reading The variable to store the reading.
   *
  */

  void getReading(rokubimini::Reading& reading);

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
   * @fn bool setCommunicationSetup()
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

private:
  /**
   * @fn bool startPollingThread()
   *
   * @brief This method starts up the polling thread.
   *
   * @return True if the operation was
   * successful.
   *
  */

  bool startPollingThread();

  /**
   * @fn bool writeCommand(const std::string& str)
   *
   * @brief Writes an Ascii string to the serial device.
   *
   * @param str The string to write.
   *
   * @return True if the operation was successful.
  */
  bool writeCommand(const std::string& str);

  /**
   * @fn bool sendCalibrationMatrixEntry(const uint8_t subId, const double entry)
   *
   * @brief Sends a calibration matrix entry to device.
   *
   * @param subId The sub-index of the SDO to write to.
   * @param entry The entry on the matrix.
   * @return True if the operation was successful.
   *
  */

  bool sendCalibrationMatrixEntry(const uint8_t subId, const double entry)
  {
    return false;
  }

  /************ rokubimini_interface methods **************/

  /**
   * @fn bool connect()
   *
   * @brief Connects to the Serial-over-USB port.
   *
   * @return True if connection was
   * successful.
   *
  */

  bool connect();

  /**
   * @fn bool connect(const std::string &port)
   *
   * @brief Connects to the Serial-over-USB port.
   *
   * @param port The port to connect to.
   * @return True if connection was
   * successful.
   *
  */

  bool connect(const std::string& port);

  /**
   * @fn bool readDevice(RxFrame &frame)
   *
   * @brief Reads a raw measurement frame from the serial-port
   *
   * @param frame The raw measurement frame to read.
   * @return True if the reading was
   * successful.
   *
  */

  //!
  bool readDevice(RxFrame& frame);

  /**
   * @fn bool isConnected() const
   *
   * @brief Checks if the device is connected.
   *
   * @return True if the device is connected.
   *
  */

  //! Quick sensor/state testers
  bool isConnected() const;

  /**
   * @fn bool isConnecting() const
   *
   * @brief Checks if device is already in the process of connecting.
   *
   * @return True if device is in the process of connecting.
   *
  */

  bool isConnecting() const;

  /**
   * @fn bool hasError() const
   *
   * @brief Checks the connection has errors.
   *
   * @return True if the connection has errors.
   *
  */

  bool hasError() const;

  /**
   * @fn bool isInConfigMode() const
   *
   * @brief Checks if the ModeState is Config Mode.
   *
   * @return True if the ModeState is Config Mode.
   *
  */

  bool isInConfigMode() const;

  //! Retreive sensor state/status

  /**
   * @fn ConnectionState getConnectionState() const
   *
   * @brief Gets the current connection status.
   *
   * @return The current connection status.
   *
  */

  ConnectionState getConnectionState() const;

  /**
   * @fn ErrorState getErrorState() const
   *
   * @brief Gets the current error status.
   *
   * @return The current error status.
   *
  */

  ErrorState getErrorState() const;

  /**
   * @fn std::string getErrorString() const
   *
   * @brief Retrieves detailed error indication.
   *
   * @return The error string.
   *
  */

  std::string getErrorString() const;

  //! Initializatin-time helper functions

  /**
   * @fn bool initSensorCommunication(bool keepConnecting)
   *
   * @brief Initializes communication with the sensor.
   *
   * @param keepConnecting Set if there will be multiple attempts
   * or only one.
   * @return True if initialization of the communication was
   * successful.
   *
  */

  bool initSensorCommunication(bool keepConnecting);

  /**
   * @fn std::uint32_t getBaudRateDefinition(const uint32_t& baudRate)
   *
   * @brief Returns the system definition for the given @a baudRate.
   *
   * @param baudRate The baudRate to find the definition for.
   * @return The system definition.
   *
  */

  std::uint32_t getBaudRateDefinition(const uint32_t& baudRate);

  /**
   * @fn bool initSerialPort(const std::string &port)
   *
   * @brief Sets up and initializes the serial port for
   * communication.
   *
   * @param port The port to initialize.
   * @return True if the port was initialized
   * successfully.
   *
  */

  bool initSerialPort(const std::string& port);

  /**
   * @fn bool initCalibrations()
   *
   * @brief Initializes calibrations of the sensor.
   *
   * @return True if calibration was
   * successful.
   *
  */

  bool initCalibrations();

  /**
   * @fn uint16_t calcCrc16_x25(uint8_t *data, int len)
   *
   * @brief Calculates the CRC16 X25 checksum for the input data.
   *
   * @param data The input data.
   * @param len The length of the input data in bytes.
   * @return The checksum calculated.
   *
  */

  uint16_t calcCrc16X25(uint8_t* data, int len);

  /**
   * @fn uint16_t crcCcittUpdate(uint16_t crc, uint8_t data)
   *
   * @brief Implementation function of the CRC16 X25 checksum for the input data.
   *
   * @param data The input data.
   * @param crc The current checksum.
   * @return The new checksum calculated.
   *
  */

  uint16_t crcCcittUpdate(uint16_t crc, uint8_t data);

  /**
   * @fn void connectionWorker()
   *
   * @brief Worker threads for managing sensor connections.
   *
  */
  //!
  void connectionWorker();

  /**
   * @fn void pollingWorker()
   *
   * @brief Worker threads for polling the sensors.
   *
  */
  //!
  void pollingWorker();

  /************ rokubimini_interface methods **************/

  /**
   * @var std::string name_
   *
   * @brief Name of the sensor.
   *
  */

  std::string name_;

  /**
   * @var Reading serialImplReading_
   *
   * @brief The internal reading variable. It's used for providing
   * to client code the sensor readings, through the \a getReading
   * () function.
   *
  */

  Reading serialImplReading_;

  /**
   * @var Command command_
   *
   * @brief Internal command variable.
   *
  */

  Command command_;

  /**
   * @var std::string port_
   *
   * @brief The serial port to connect to.
   *
  */

  std::string port_;

  /**
   * @var std::uint32_t baudRate_
   *
   * @brief The baud rate of the serial communication.
   *
  */

  std::uint32_t baudRate_;

  /**
   * @var mutable std::recursive_mutex readingMutex_
   *
   * @brief Mutex prohibiting simultaneous access the internal Reading variable.
   *
  */

  mutable std::recursive_mutex readingMutex_;

  /**
   * @var mutable std::recursive_mutex serialMutex_
   *
   * @brief Mutex prohibiting simultaneous access to Serial device.
   *
  */

  mutable std::recursive_mutex serialMutex_;

  /************ rokubimini_interface variables **************/
  /*
* Device USB communication inteface
*/
  // int deviceId_;
  // boost::atomic<int> productId_;

  /**
   * @var boost::atomic<int> usbFileDescriptor_
   *
   * @brief The USB file descriptor.
   *
  */

  boost::atomic<int> usbFileDescriptor_;

  /**
   * @var boost::atomic<bool> frameSync_
   *
   * @brief Flag that indicates if the frame is synced.
   *
  */

  boost::atomic<bool> frameSync_;

  /**
   * @var std::ifstream usbStreamIn_
   *
   * @brief Input stream for the USB file descriptor.
   *
  */

  std::ifstream usbStreamIn_;

  /**
   * @var std::ofstream usbStreamOut_
   *
   * @brief Output stream for the USB file descriptor.
   *
  */

  std::ofstream usbStreamOut_;

  /**
   * @var uint8_t frameHeader
   *
   * @brief The frame header value.
   *
  */

  uint8_t frameHeader = 0xAA;

  /**
   * @var RxFrame frame_
   *
   * @brief The internal variable for the receiving frame. This variable represents the raw data received from the
   * sensor.
   *
  */

  RxFrame frame_;

  /*
   *  Device measurements and data buffers
  */

  /**
   * @var double timeStampSecs_
   *
   * @brief Sample time-stamp.
   *
  */
  double timeStampSecs_;

  //! Latest low-pass filtered sample (after adjustment for calibrations)
  // Wrench wrenchFiltered_;
  //! Latest measurement sample (after adjustment for calibrations)
  // Wrench wrenchMeasured_;
  //! Latest measurement sample (after adjustment for calibrations)
  // Wrench wrenchMeasuredCompensated_;

  // boost::shared_mutex mutexWrenchMeasured_;
  // boost::shared_mutex mutexWrenchMeasuredCompensated_;

  /*
* Device calibrations
*/

  // bool isCalibrated_;

  // //! The offset calibration
  // WrenchRaw offsetWrenchRaw_;
  // boost::shared_mutex mutexWrenchRawOffset_;

  // //! The sensor calibration matrix
  // Matrix6D calibrationMatrix_;
  // boost::shared_mutex mutexCalibrationMatrix_;

  // //! The bias calibration
  // Wrench biasWrench_;
  // boost::shared_mutex mutexBiasWrench_;
  // //! The bias calibration
  // Wrench biasWrenchCompensated_;
  // boost::shared_mutex mutexBiasWrenchCompensated_;

  // //! The temperature compensation
  // Wrench temperatureCalibration_;
  // boost::shared_mutex mutexTemperatureCalibration_;

  /*
   *  Driver internals
  */

  //! First-order, low-pass (discrete-time IIR) filter coefficient
  // double lowPassFilterCoefficient_;

  /**
   * @var bool runInThreadedMode_
   *
   * @brief Flag to indicate whether the driver should setup worker
   * threads at startup.
   *
  */
  bool runInThreadedMode_;

  /**
   * @var double pollingThreadUpdateRateSecs_
   *
   * @brief If setup, the sensor polling thread will poll at this
   * rate.
   *
  */

  double pollingThreadUpdateRateSecs_;

  /**
   * @var bool useDeviceTimeStamps_
   *
   * @brief Flag to indicate whether time-stamps should be
   * generated using sensor or system time.
   *
  */

  bool useDeviceTimeStamps_;

  /**
   * @var boost::thread connectionThread_
   *
   * @brief Connection thread handle.
   *
  */

  boost::thread connectionThread_;

  /**
   * @var boost::thread pollingThread_
   *
   * @brief Polling thread handle.
   *
  */

  boost::thread pollingThread_;

  /**
   * @var boost::atomic<ConnectionState> connectionState_
   *
   * @brief Internal connection state.
   *
  */

  boost::atomic<ConnectionState> connectionState_;

  /**
   * @var boost::atomic<ErrorState> errorState_
   *
   * @brief Internal error state.
   *
  */
  boost::atomic<ErrorState> errorState_;

  /**
   * @var boost::atomic<ModeState> modeState_
   *
   * @brief Mode state of the sensor.
   *
  */

  boost::atomic<ModeState> modeState_;
  /**
   * @var boost::atomic<bool> isRunning_
   *
   * @brief Internal flag to indicate if the threads are running.
   *
  */
  //! Internal flags/indicators
  boost::atomic<bool> isRunning_;
  // boost::atomic<bool> isReceivingOffset_;

  /**
   * @var unsigned long pollingSyncErrorCounter_
   *
   * @brief Synchronization error counter.
   *
  */
  //! Internal statistics and error counters
  unsigned long pollingSyncErrorCounter_;

  /**
   * @var unsigned long frameReceivedCounter_
   *
   * @brief Received frame counter.
   *
  */
  unsigned long frameReceivedCounter_;

  /**
   * @var unsigned long frameSuccessCounter_
   *
   * @brief Correct frames counter.
   *
  */

  unsigned long frameSuccessCounter_;

  /**
   * @var unsigned long frameCrcErrorCounter_
   *
   * @brief Counter for frames with CRC errors.
   *
  */

  unsigned long frameCrcErrorCounter_;

  /**
   * @var unsigned int frameSyncErrorCounter_
   *
   * @brief Frame sync errors.
   *
  */
  unsigned int frameSyncErrorCounter_;

  /**
   * @var unsigned int maxFrameSyncErrorCounts_
   *
   * @brief Maximum acceptable frame sync errors.
   *
  */
  unsigned int maxFrameSyncErrorCounts_;
  /************ rokubimini_interface variables **************/
};

using RokubiminiSerialImplPtr = std::shared_ptr<RokubiminiSerialImpl>;

}  // namespace serial
}  // namespace rokubimini
