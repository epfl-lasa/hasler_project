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

//! System dependencies
#include <stdint.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <cerrno>
#include <fstream>
#include <functional>
#include <thread>

#include <asm/ioctls.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
#include <string>

#include "ros/ros.h"
//! Library dependencies
#include <message_logger/message_logger.hpp>

//! Class header file
#include <rokubimini_serial/RokubiminiSerialImpl.hpp>

namespace rokubimini
{
namespace serial
{
RokubiminiSerialImpl::RokubiminiSerialImpl(const std::string& name, const std::string& port,
                                           const std::uint32_t& baudRate)
  : name_(name)
  , port_(port)
  , baudRate_(baudRate)
  , usbFileDescriptor_{ -1 }
  , frameSync_{ false }
  , usbStreamIn_{ nullptr }
  , usbStreamOut_{ nullptr }
  , timeStampSecs_(0.0)
  , runInThreadedMode_(true)
  , pollingThreadUpdateRateSecs_(0.0025)
  , useDeviceTimeStamps_(false)
  , connectionThread_()
  , pollingThread_()
  , connectionState_(ConnectionState::DISCONNECTED)
  , errorState_(ErrorState::NO_ERROR)
  , modeState_{ ModeState::RUN_MODE }
  , isRunning_{ true }
  , pollingSyncErrorCounter_(0)
  , frameReceivedCounter_(0)
  , frameSuccessCounter_(0)
  , frameCrcErrorCounter_(0)
  , frameSyncErrorCounter_(0)
  , maxFrameSyncErrorCounts_(10000)
{
  MELO_DEBUG("[%s] Device connected to serial port: %s with baudrate: %u \n", name_.c_str(), port_.c_str(), baudRate_);
  ros::Time::init();
}

bool RokubiminiSerialImpl::init()
{
  MELO_DEBUG("[%s] Attempting to start-up device.", name_.c_str());

  // Connect to serial port
  if (connect())
  {
    // Sleep for 3.0 seconds for the sensor to boot and start sending data.
    std::this_thread::sleep_for(std::chrono::microseconds(3000000));
    MELO_INFO("[%s] Start-up successful.", name_.c_str());
  }
  else
  {
    MELO_ERROR("[%s] Could not establish connection with device. Start-up failed.", name_.c_str());
    return false;
  }
  return true;
}

bool RokubiminiSerialImpl::startPollingThread()
{
  // Start the polling thread if driver is to run in threaded mode and if
  // the polling thread is not already running
  if (runInThreadedMode_ && !pollingThread_.joinable())
  {
    MELO_INFO("[%s] Launching polling thread.", name_.c_str());
    pollingThread_ = boost::thread{ &RokubiminiSerialImpl::pollingWorker, this };
  }
  return true;
}

bool RokubiminiSerialImpl::startup()
{
  return startPollingThread();
}

void RokubiminiSerialImpl::shutdown()
{
  MELO_INFO("[%s] Driver will attempt to shut-down", name_.c_str());

  // Signal for termination (all background thread)
  isRunning_ = false;

  // Shutdown the connection thread if running
  if (runInThreadedMode_ && connectionThread_.joinable())
  {
    connectionThread_.join();
  }

  // Shutdown the polling thread if running
  if (runInThreadedMode_ && pollingThread_.joinable())
  {
    pollingThread_.join();
  }

  // Close the serial-port file descriptior
  if (usbFileDescriptor_ != -1)
  {
    MELO_INFO("[%s] Closing Serial Communication", name_.c_str());
    close(usbFileDescriptor_);
  }
  usbStreamIn_.close();
  usbStreamOut_.close();
  if (((usbStreamIn_.fail() | usbStreamOut_.fail()) != 0))
  {
    MELO_ERROR("[%s] Failed to close file streams.", name_.c_str());
  }

  MELO_INFO("[%s] Shut-down successful", name_.c_str());
}

bool RokubiminiSerialImpl::connect()
{
  // Abort attempt if the driver is already in the process of connecting
  if (connectionState_ == ConnectionState::ISCONNECTING)
  {
    return false;
  }
  else
  {
    connectionState_ = ConnectionState::ISCONNECTING;
  }

  // Close the file descriptor if it was already open
  if (usbFileDescriptor_ != -1)
  {
    close(usbFileDescriptor_);
    usbFileDescriptor_ = -1;
  }

  // Reset error counters and error state
  frameReceivedCounter_ = 0;
  frameSuccessCounter_ = 0;
  frameCrcErrorCounter_ = 0;
  frameSyncErrorCounter_ = 0;
  errorState_ = ErrorState::NO_ERROR;

  // Check if the driver is set to run in threaded mode
  if (!runInThreadedMode_)
  {
    return initSensorCommunication(false);
  }
  else
  {
    // Start the connection thread
    connectionThread_ = boost::thread{ &RokubiminiSerialImpl::connectionWorker, this };
  }

  return true;
}

bool RokubiminiSerialImpl::connect(const std::string& port)
{
  port_ = port;
  return connect();
}

bool RokubiminiSerialImpl::readDevice(RxFrame& frame)
{
  while (errorState_ == ErrorState::NO_ERROR && isRunning_ && modeState_ == ModeState::RUN_MODE)
  {
    /* read the next available byte and check if is the header
 * make sure to unget it after. emulating peek. Using static
 * variable for the sync flag will preserve status of sync
 * between calls */
    while (!frameSync_)
    {
      uint8_t possible_header;
      /* read bytes 1 by 1 to find the header */
      usbStreamIn_.read((char*)&possible_header, sizeof(possible_header));
      if (possible_header == frameHeader)
      {
        /* read the remaining frame to make check also CRC */
        usbStreamIn_.read((char*)&frame.bytes[sizeof(frame.header)], sizeof(frame) - sizeof(frame.header));
        if (frame.crc16_ccitt == calcCrc16X25(frame.data.bytes, sizeof(frame.data)))
        {
          MELO_INFO("[%s] Frame synced with 0x%X header", name_.c_str(), frameHeader);
          frameSync_ = true;
        }
        else
        {
          /* if there is a frame that included the header 0xAA in
           * a fixed position. Could be the above checking mechanism
           * will get stuck because will find the wrong value as header
           * then will remove from the buffer n bytes where n the size
           * of the frame and then will find again exactly the same
           * situation the wrong header. So we read on extra byte to make
           * sure next time will start from the position that is size of frame
           * plus 1. It works */
          char dummy;
          // usbStreamIn_.read((char*)&dummy, sizeof(dummy));
          usbStreamIn_.read((char*)&dummy, sizeof(dummy));
          MELO_WARN("[%s] Skipping incomplete frame", name_.c_str());
        }
      }
    }
    /* Read the sensor measurements frame assuming that is alligned with the RX buffer */
    try
    {
      usbStreamIn_.read((char*)frame.bytes, sizeof(frame));
    }
    catch (std::exception& e)
    {
      MELO_ERROR("[%s] Error while reading a packet, %s", name_.c_str(), e.what());
      errorState_ = ErrorState::PACKET_READING_ERROR;
      return false;
    }

    /* Check if the frame is still alligned, otherwise exit */
    if (frame.header != frameHeader)
    {
      frameSync_ = false;
      // char buf[200];
      // usbStreamIn_.read(buf, 1); //consume the peeked charachters
      // MELO_WARN(buf);
      /* keep some statistics */
      if (++frameSyncErrorCounter_ >= maxFrameSyncErrorCounts_)
      {
        MELO_WARN("[%s] Reached max syncing errors. Disconnecting sensor.", name_.c_str());
        errorState_ = ErrorState::SYNC_ERROR;
        connectionState_ = ConnectionState::DISCONNECTED;
      }
      return false;
    }
    // Read and check CRC 16-bit
    try
    {
      uint16_t crc_received = frame.crc16_ccitt;
      uint16_t crc_computed = calcCrc16X25(frame.data.bytes, sizeof(frame.data));
      if (crc_received != crc_computed)
      {
        frameCrcErrorCounter_++;
        MELO_WARN("[%s] CRC missmatch received: 0x%04x calculated: 0x%04x", name_.c_str(), crc_received, crc_computed);
        return false;
      }
    }
    catch (std::exception& e)
    {
      MELO_ERROR("[%s] Error while reading a packet, %s", name_.c_str(), e.what());
      errorState_ = ErrorState::PACKET_READING_ERROR;
      return false;
    }

    if (frame.data.status.app_took_too_long)
    {
      MELO_WARN("[%s] Warning force sensor is skipping measurements, Increase sinc length", name_.c_str());
    }
    if (frame.data.status.overrange)
    {
      MELO_WARN("[%s] Warning measuring range exceeded", name_.c_str());
    }
    if (frame.data.status.invalid_measurements)
    {
      MELO_ERROR("[%s] Warning force torque measurements are invalid, Permanent damage may occur", name_.c_str());
    }
    if (frame.data.status.raw_measurements)
    {
      MELO_WARN_THROTTLE(1.0, "[%s] Warning raw force torque measurements from gages", name_.c_str());
    }

    // sample.getForce().x() = frame.data.forces[0];
    // sample.getForce().y() = frame.data.forces[1];
    // sample.getForce().z() = frame.data.forces[2];
    // sample.getTorque().x() = frame.data.forces[3];
    // sample.getTorque().y() = frame.data.forces[4];
    // sample.getTorque().z() = frame.data.forces[5];

    // tempCount = frame.data.temperature;
    // timeStamp = frame.data.timestamp;
    frameSuccessCounter_++;
    return true;
  }
  // This point should not be reached
  return false;
}

bool RokubiminiSerialImpl::isConnected() const
{
  return (connectionState_ == ConnectionState::CONNECTED);
}

bool RokubiminiSerialImpl::isConnecting() const
{
  return (connectionState_ == ConnectionState::ISCONNECTING);
}

bool RokubiminiSerialImpl::hasError() const
{
  return (errorState_ != ErrorState::NO_ERROR);
}

bool RokubiminiSerialImpl::isInConfigMode() const
{
  return (modeState_ == ModeState::CONFIG_MODE);
}

ConnectionState RokubiminiSerialImpl::getConnectionState() const
{
  return connectionState_;
}

ErrorState RokubiminiSerialImpl::getErrorState() const
{
  return errorState_;
}

std::string RokubiminiSerialImpl::getErrorString() const
{
  switch (errorState_)
  {
    case ErrorState::NO_ERROR:
      return std::string{ "No Error" };

    case ErrorState::OFFSET_ERROR:
      return std::string{ "Offset Error" };

    case ErrorState::CALIBRATION_ERROR:
      return std::string{ "Calibration Error" };

    case ErrorState::PACKET_READING_ERROR:
      return std::string{ "Packet Reading Error" };

    case ErrorState::SYNC_ERROR:
      return std::string{ "Sync Error" };

    default:
      return std::string{ "Undefined" };
  }

  return std::string{ "No Error" };
}

bool RokubiminiSerialImpl::initSensorCommunication(bool keepConnecting)
{
  MELO_INFO("[%s] Initializing serial-port at: %s", name_.c_str(), port_.c_str());

  // Block, continuously trying to connect to the serial-port, until
  // signaled to stop or connection is established
  bool connected;
  do
  {
    connected = initSerialPort(port_);
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  } while (isRunning_ && keepConnecting && !connected);

  if (!isRunning_)
  {
    // connection process was cancelled from somewhere else
    MELO_ERROR("[%s] Sensor is disconnected.", name_.c_str());
    connectionState_ = ConnectionState::DISCONNECTED;
    return false;
  }

  connectionState_ = ConnectionState::CONNECTED;
  return true;
}

uint32_t RokubiminiSerialImpl::getBaudRateDefinition(const uint32_t& baudRate)
{
  switch (baudRate)
  {
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      MELO_ERROR("[%s] Baud rate %u is not supported, setting it 0\n", name_.c_str(), baudRate);
      return B0;
  }
}
bool RokubiminiSerialImpl::initSerialPort(const std::string& port)
{
  // int flags;
  termios newtio;
  struct serial_struct ser_info;

  // Open the Serial Port
  usbFileDescriptor_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (usbFileDescriptor_ < 0)
  {
    MELO_ERROR("[%s] Failed to open serial-port: '%s'", name_.c_str(), port.c_str());
    return false;
  }

  // Get the Serial Descriptor Flags
  if (tcgetattr(usbFileDescriptor_, &newtio) < 0)
  {
    MELO_ERROR("[%s] Failed to get connection attributes.", name_.c_str());
    return false;
  }

  // Set the Serial Speed
  cfsetispeed(&newtio, getBaudRateDefinition(baudRate_));
  cfsetospeed(&newtio, getBaudRateDefinition(baudRate_));
  cfmakeraw(&newtio);

  // Set the Serial Connection Attributes (SCA)
  if (tcsetattr(usbFileDescriptor_, TCSAFLUSH, &newtio) < 0)
  {
    MELO_ERROR("[%s] Failed to set connection attributes.", name_.c_str());
    return false;
  }

  // Flush the input and output streams
  if (tcflush(usbFileDescriptor_, TCIOFLUSH) < 0)
  {
    MELO_ERROR("[%s] Failed to flush the input and output streams.", name_.c_str());
    return false;
  }

  // Enable linux FTDI low latency mode
  ioctl(usbFileDescriptor_, TIOCGSERIAL, &ser_info);
  ser_info.flags |= ASYNC_LOW_LATENCY;
  ioctl(usbFileDescriptor_, TIOCSSERIAL, &ser_info);

  // Get the Serial Descriptor Flags
  if (fcntl(usbFileDescriptor_, F_GETFL) < 0)
  {
    MELO_ERROR("[%s] Failed to set the descriptor flags.", name_.c_str());
    return false;
  }

  // create a std stream to read and write from/to device
  usbStreamIn_.open(port.c_str(), std::ifstream::in);
  usbStreamOut_.open(port.c_str(), std::ofstream::out);
  if (((usbStreamIn_.fail() | usbStreamOut_.fail()) != 0))
  {
    MELO_ERROR("[%s] Failed to open file streams.", name_.c_str());
    return false;
  }
  // usbStreamIn_.rdbuf()->pubsetbuf(Buffer, N);

  // wait another second to get measurements
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return true;
}

bool RokubiminiSerialImpl::initCalibrations()
{
  bool ret = true;

  // if (!isCalibrated_)
  // {
  // if (this->configuration_.getSensorCalibration().getCalibrationMatrix() == Eigen::Matrix<double, 6, 6>::Zero()) //
  // TODO
  // {
  //   // const auto matrix = Eigen::Matrix<double, 6, 6>::Identity();
  //   // this->configuration_.getSensorCalibration().setCalibrationMatrix(matrix);
  //   errorState_ = ErrorState::CALIBRATION_ERROR;
  //   ret = false;
  // }

  // if (this->configuration_.getForceTorqueOffset() == Eigen::Matrix<double, 6, 1>::Zero()) // TODO
  // {
  //   errorState_ = ErrorState::OFFSET_ERROR;
  //   ret = false;
  // }

  // if (!this->loadBiasWrenchFromFile(biasWrenchCalibrationsFile_)) // TODO
  // {
  //   biasWrench_.setZero();
  // }
  // }

  return ret;
}

inline uint16_t RokubiminiSerialImpl::calcCrc16X25(uint8_t* data, int len)
{
  uint16_t crc = 0xFFFF;
  while (len--)
    crc = crcCcittUpdate(crc, *data++);
  return ~crc;
}

uint16_t RokubiminiSerialImpl::crcCcittUpdate(uint16_t crc, uint8_t data)
{
#define lo8(x) ((x)&0xFF)
#define hi8(x) (((x) >> 8) & 0xFF)
  data ^= lo8(crc);
  data ^= data << 4;

  return ((((uint16_t)data << 8) | hi8(crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

void RokubiminiSerialImpl::connectionWorker()
{
  initSensorCommunication(true);
}

void RokubiminiSerialImpl::pollingWorker()
{
  // Configure the scheduling policy for the pollin gthread
  int priority = 99;
  sched_param params;
  params.sched_priority = priority;
  if (sched_setscheduler(getpid(), SCHED_FIFO, &params) != 0)
  {
    MELO_WARN("[%s] Failed to set thread priority to %d: %s.", name_.c_str(), priority, std::strerror(errno));
  }

  // Wait for connection to be established before starting to poll
  usleep(100000);
  if (!isConnected())
  {
    while (!isConnected() && isRunning_)
    {
      usleep(1000);
      MELO_WARN_THROTTLE(4.0, "[%s] Polling thread waiting for connection. ", name_.c_str());
    }
  }
  // Declare and initialize buffers
  // Wrench receviedWrenchRaw;
  // WrenchRaw offsetWrenchRaw;
  // Wrench measuredWrenchRaw;
  // Wrench tempInit;
  // Wrench tempComp;
  // Wrench measuredWrench;
  // Wrench filteredWrench;
  // Wrench biasWrench;
  double time_stamp = 0.0;
  // unsigned int deviceTimeStamp = 0;
  // unsigned int deviceId = 0;
  // unsigned int tempRaw = 0;

  // Variables for timing
  const double update_rate_hz = 1.0 / pollingThreadUpdateRateSecs_;
  struct timespec tnow, tloop, tduration;
  bool run_processing = true;
  unsigned long loop_counter = 0;
  pollingSyncErrorCounter_ = 0;

  // Initialize timers
  // NOTE: tnow is always measured from the clock, while tloop is incremented
  // according to the sampling rate. If they are different at each loop iteration
  // then processing might be slow, causing sensor polling to be of the
  // synchronization

  clock_gettime(CLOCK_MONOTONIC_RAW, &tnow);
  tloop = tnow;
  tduration.tv_sec = 0;
  tduration.tv_nsec = 0;

  // Start sensor polling
  while (isRunning_ && modeState_ == ModeState::RUN_MODE)
  {
    // Measure loop periodicity
    //    temp = tperiod;
    //    clock_gettime(CLOCK_MONOTONIC_RAW, &tperiod);
    //    tperiod.tv_nsec -= temp.tv_nsec;
    //    tperiod.tv_sec -= temp.tv_sec;

    // Check if loop is out of synchronization
    double time_delta = (double)(tduration.tv_sec) + (double)(tduration.tv_nsec) * 1e-9;
    if (fabs(time_delta) > 5e-3)
    {
      pollingSyncErrorCounter_++;
      MELO_WARN("[%s] Polling is out of sync! TimeDelta=%f", name_.c_str(), time_delta);
    }

    // Poll the serial device, skipping processing in current iteration if no valid data has been received
    RxFrame frame;

    std::unique_lock<std::recursive_mutex> lock(serialMutex_);
    (this->readDevice(frame)) ? (run_processing = true) : (run_processing = false);
    lock.unlock();
    // TODO: check if PID of sent data and PID configured match
    // We have to check the connection again because readDevice() may have signalled to disconnect
    // if too many communication errors occurred
    if (connectionState_ == ConnectionState::CONNECTED && run_processing)
    {
      // Run processing steps:
      /* 1. Correct for device-specific offset calibrations */
      // Wrench OffsetWrenchRaw_(offsetWrenchRaw_.getVector().cast<double>());

      // measuredWrenchRaw = receviedWrenchRaw - OffsetWrenchRaw_;

      /* 2. Multiply with calibration matrix */
      // measuredWrench = Wrench(calibrationMatrix_ * measuredWrenchRaw.getVector().cast<double>());
      // MELO_WARN_THROTTLE_STREAM(0.1,"\n\nMatrix:\n" << calibrationMatrix_);
      /* 3. Correct for system-specific biases */
      // measuredWrench -= biasWrench_;
      /* 4. Perform low-pass filtering */
      // filteredWrench = measuredWrench * lowPassFilterCoefficient_ + filteredWrench * (1.0 -
      // lowPassFilterCoefficient_);

      /* 5. Set the time-stamp */
      if (useDeviceTimeStamps_)
      {
        time_stamp = (double)frame.data.timestamp * 1e-6;
      }
      else
      {
        time_stamp = static_cast<double>(tnow.tv_sec) + 1.0e-9 * static_cast<double>(tnow.tv_nsec);
      }
      // Store the final measurement into the primary buffer
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        // boost::unique_lock<boost::shared_mutex> lock{mutexWrenchMeasured_};
        frame_ = frame;
        // wrenchMeasured_ = measuredWrench;
        // wrenchFiltered_ = filteredWrench;
        timeStampSecs_ = time_stamp;
      }
    }

    // Measure loop duration
    clock_gettime(CLOCK_MONOTONIC_RAW, &tduration);
    tduration.tv_nsec -= tnow.tv_nsec;
    tduration.tv_sec -= tnow.tv_sec;

    // Increment the time predicted time of the loop
    tloop.tv_nsec += (int)(1e+9 / (double)(update_rate_hz)) - tduration.tv_nsec;
    tloop.tv_sec += (int)(tloop.tv_nsec * 1e-9) - tduration.tv_sec;
    tloop.tv_nsec = tloop.tv_nsec % 1000000000;

    // Sleep to implement cyclic polling
    clock_nanosleep(CLOCK_MONOTONIC_RAW, TIMER_ABSTIME, &tloop, nullptr);

    // Measure the current absolute time
    clock_gettime(CLOCK_MONOTONIC_RAW, &tnow);
    loop_counter++;
  }

  // Show error statistics if errors have been detected
  if ((pollingSyncErrorCounter_ > 0) || (frameSyncErrorCounter_ > 0) || (frameCrcErrorCounter_ > 0))
  {
    MELO_WARN("[%s] Sensor polling thread error report:", name_.c_str());
    MELO_WARN("[%s]   Total iterations:    %lu", name_.c_str(), loop_counter);
    MELO_WARN("[%s]   Frames received:     %lu, (%f%%)", name_.c_str(), frameReceivedCounter_,
              100.0 * ((double)frameReceivedCounter_ / (double)loop_counter));
    MELO_WARN("[%s]   Correct frames:      %lu, (%f%%)", name_.c_str(), frameSuccessCounter_,
              100.0 * ((double)frameSuccessCounter_ / (double)frameReceivedCounter_));
    MELO_WARN("[%s]   Polling sync errors: %lu, (%f%%)", name_.c_str(), pollingSyncErrorCounter_,
              100.0 * ((double)pollingSyncErrorCounter_ / (double)loop_counter));
    MELO_WARN("[%s]   Frame CRC errors:    %lu, (%f%%)", name_.c_str(), frameCrcErrorCounter_,
              100.0 * ((double)frameCrcErrorCounter_ / (double)frameReceivedCounter_));
    MELO_WARN("[%s]   Frame sync errors:   %d", name_.c_str(), frameSyncErrorCounter_);
  }
}

void RokubiminiSerialImpl::getReading(rokubimini::Reading& reading)
{
  if (isConnected())
  {
    // My implementation: Ilias please review/change at your will
    {
      std::lock_guard<std::recursive_mutex> lock(readingMutex_);

      const auto& stamp = any_measurements::Time::nowWallClock();

      serialImplReading_.getWrench().time_ = stamp;
      serialImplReading_.getWrench().wrench_.getForce().x() = frame_.data.forces[0];
      serialImplReading_.getWrench().wrench_.getForce().y() = frame_.data.forces[1];
      serialImplReading_.getWrench().wrench_.getForce().z() = frame_.data.forces[2];
      serialImplReading_.getWrench().wrench_.getTorque().x() = frame_.data.forces[3];
      serialImplReading_.getWrench().wrench_.getTorque().y() = frame_.data.forces[4];
      serialImplReading_.getWrench().wrench_.getTorque().z() = frame_.data.forces[5];

      serialImplReading_.getImu().time_ = stamp;
      // serialImplReading_.getImu().angularVelocity_.x() = frame_.data.imu[0];
      // serialImplReading_.getImu().angularVelocity_.y() = frame_.data.imu[1];
      // serialImplReading_.getImu().angularVelocity_.z() = frame_.data.imu[2];
      // serialImplReading_.getImu().linearAcceleration_.x() = frame_.data.imu[3];
      // serialImplReading_.getImu().linearAcceleration_.y() = frame_.data.imu[4];
      // serialImplReading_.getImu().linearAcceleration_.z() = frame_.data.imu[5];

      // serialImplReading_.setStatusword(Statusword(txPdoA.warningsErrorsFatals_));
      reading = serialImplReading_;
    }
  }
}

bool RokubiminiSerialImpl::setConfigMode()
{
  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand("C");
  if (!success)
  {
    lock.unlock();
    return false;
  }
  else
  {
    modeState_ = ModeState::CONFIG_MODE;
    lock.unlock();

    std::this_thread::sleep_for(std::chrono::microseconds(500000));
    if (runInThreadedMode_ && pollingThread_.joinable())
    {
      pollingThread_.join();
    }
    // Reset error counters and error state
    frameReceivedCounter_ = 0;
    frameSuccessCounter_ = 0;
    frameCrcErrorCounter_ = 0;
    frameSyncErrorCounter_ = 0;
    errorState_ = ErrorState::NO_ERROR;
    return true;
  }
}

bool RokubiminiSerialImpl::setRunMode()
{
  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand("R");
  lock.unlock();
  if (!success)
  {
    return false;
  }

  // Start a new polling thread.
  modeState_ = ModeState::RUN_MODE;
  if (runInThreadedMode_ && !pollingThread_.joinable())
  {
    MELO_INFO("[%s] Launching polling thread.", name_.c_str());
    pollingThread_ = boost::thread{ &RokubiminiSerialImpl::pollingWorker, this };
  }
  return true;
}

bool RokubiminiSerialImpl::setHardwareReset()
{
  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand("#");
  lock.unlock();
  return success;
}

bool RokubiminiSerialImpl::setSoftwareReset()
{
  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand("I");
  if (!success)
  {
    lock.unlock();
    return false;
  }
  else
  {
    modeState_ = ModeState::CONFIG_MODE;
    lock.unlock();

    std::this_thread::sleep_for(std::chrono::microseconds(500000));
    if (runInThreadedMode_ && pollingThread_.joinable())
    {
      pollingThread_.join();
    }
    // Reset error counters and error state
    frameReceivedCounter_ = 0;
    frameSuccessCounter_ = 0;
    frameCrcErrorCounter_ = 0;
    frameSyncErrorCounter_ = 0;
    errorState_ = ErrorState::NO_ERROR;
    return true;
  }
}

bool RokubiminiSerialImpl::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  if (!isInConfigMode())
  {
    return false;
  }

  MELO_DEBUG("[%s] Setting force/torque filter", name_.c_str());
  MELO_DEBUG("[%s] \tsize: %u", name_.c_str(), filter.getSincFilterSize());
  MELO_DEBUG("[%s] \tchop: %u", name_.c_str(), filter.getChopEnable());
  MELO_DEBUG("[%s] \tfast: %u", name_.c_str(), filter.getFastEnable());
  MELO_DEBUG("[%s] \tskip: %u", name_.c_str(), filter.getSkipEnable());

  char buffer[100];
  int ret = sprintf(buffer, "f,%u,%u,%u,%u", filter.getSincFilterSize(), filter.getChopEnable(), filter.getFastEnable(),
                    filter.getSkipEnable());

  if (ret < 0)
  {
    MELO_ERROR("[%s] sprintf failed to write data", name_.c_str());
    return false;
  }
  std::string str(buffer);
  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand(str);
  lock.unlock();
  return success;
}

bool RokubiminiSerialImpl::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  if (!isInConfigMode())
  {
    return false;
  }

  MELO_DEBUG_STREAM("[" << name_.c_str() << "] Setting Force/Torque offset: " << forceTorqueOffset << std::endl);
  char buffer[100];
  int ret = sprintf(buffer, "b,%f,%f,%f,%f,%f,%f", forceTorqueOffset(0, 0), forceTorqueOffset(1, 0),
                    forceTorqueOffset(2, 0), forceTorqueOffset(3, 0), forceTorqueOffset(4, 0), forceTorqueOffset(5, 0));

  if (ret < 0)
  {
    MELO_ERROR("[%s] sprintf failed to write data", name_.c_str());
    return false;
  }
  std::string str(buffer);
  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand(str);
  lock.unlock();
  return success;
}

bool RokubiminiSerialImpl::setCommunicationSetup(const configuration::SensorConfiguration& sensorConfiguration,
                                                 const uint8_t& dataFormat, const uint32_t& baudRate)
{
  if (!isInConfigMode())
  {
    return false;
  }

  MELO_DEBUG("[%s] Setting sensor configuration & communication setup", name_.c_str());
  uint8_t baud_rate;
  switch (baudRate)
  {
    case 9600:
      baud_rate = 0;
      break;
    case 57600:
      baud_rate = 1;
      break;
    case 115200:
      baud_rate = 2;
      break;
    case 230400:
      baud_rate = 3;
      break;
    case 460800:
      baud_rate = 4;
      break;
    default:
      MELO_ERROR("[%s] The baud rate %u is not supported\n", name_.c_str(), baudRate);
      return false;
  }

  char buffer[100];
  int ret = sprintf(buffer, "c,%u,%u,%u,%u", sensorConfiguration.getTemperatureCompensationActive(),
                    sensorConfiguration.getCalibrationMatrixActive(), dataFormat, baud_rate);

  if (ret < 0)
  {
    MELO_ERROR("[%s] sprintf failed to write data", name_.c_str());
    return false;
  }
  std::string str(buffer);
  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand(str);
  lock.unlock();
  return success;
}

bool RokubiminiSerialImpl::saveConfig()
{
  if (!isInConfigMode())
  {
    return false;
  }

  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand("s");
  lock.unlock();
  return success;
}
bool RokubiminiSerialImpl::loadConfig()
{
  if (!isInConfigMode())
  {
    return false;
  }

  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand("l");
  lock.unlock();
  return success;
}
bool RokubiminiSerialImpl::printUserConfig()
{
  if (!isInConfigMode())
  {
    return false;
  }

  std::unique_lock<std::recursive_mutex> lock(serialMutex_);
  bool success = writeCommand("w");
  lock.unlock();
  if (!success)
  {
    return false;
  }
  uint8_t c;
  uint64_t timespec_diff_ns;
  struct timespec tnow, tloop;
  clock_gettime(CLOCK_MONOTONIC_RAW, &tnow);
  tloop = tnow;
  lock.lock();
  MELO_INFO("[%s] Printing user configuration:", name_.c_str());
  do
  {
    /* run the next readsome() only if there are available bytes in the input buffer.*/
    if (usbStreamIn_.rdbuf()->in_avail() > 0)
    {
      /* read bytes 1 by 1 and print them.*/
      usbStreamIn_.readsome((char*)&c, 1);
      printf("%c", c);
    }
    clock_gettime(CLOCK_MONOTONIC_RAW, &tloop);
    timespec_diff_ns = (tloop.tv_sec - tnow.tv_sec) * 1e9 + (tloop.tv_nsec - tnow.tv_nsec);
  } while (timespec_diff_ns < 1e9);  // run the loop for 1sec.
  lock.unlock();
  return true;
}

bool RokubiminiSerialImpl::writeCommand(const std::string& str)
{
  try
  {
    MELO_DEBUG("[%s] Number of chars: %lu", name_.c_str(), str.size());
    MELO_DEBUG("[%s] String chars: %s", name_.c_str(), str.c_str());
    if (usbStreamIn_.is_open() && usbStreamOut_.is_open())
    {
      usbStreamIn_.sync();
      usbStreamOut_.write(str.c_str(), str.size());
      usbStreamOut_.flush();
      if ((usbStreamOut_.fail() | usbStreamIn_.fail()) != 0)
      {
        MELO_WARN("[%s] Serial Write or Read failed", name_.c_str());
        return false;
      }
    }
    else
    {
      MELO_WARN("[%s] Streams are not yet open.", name_.c_str());
      return false;
    }
  }
  catch (const std::exception& e)
  {
    MELO_ERROR("[%s] %s", name_.c_str(), e.what());
    return false;
  }
  return true;
}
}  // namespace serial
}  // namespace rokubimini