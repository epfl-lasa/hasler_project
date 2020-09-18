/*!
 * @file	Echo.hpp
 * @author	Philipp Leemann
 * @date	Jan, 2018
 */

#pragma once

// use the DEBUG_COSMO flag to print info on console
#ifndef COSMO_DEBUG_LEVEL2
#define COSMO_DEBUG_LEVEL2
#endif

#include "cosmo_ros/cosmo_ros.hpp"
#include "signal_handler/SignalHandler.hpp"

#include <atomic>

namespace cosmo_ros_tools
{
template <typename Msg_, typename MsgRos_, template <typename, typename> class Converter_ = cosmo_ros::ConversionTraits>
class Echo
{
public:
  using SubscriberImpl = cosmo_ros::SubscriberRosPtr<Msg_, MsgRos_, Converter_>;

  Echo() = delete;

  /*!
   * @param inTopic   Name of the topic to listen to
   * @param outTopic  Name of the topic to republish on
   */
  Echo(int argc, char** argv, const std::string& inTopic) : running_{ false }
  {
    ros::init(argc, argv, "Echo", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    auto subOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<Msg_>>(
        inTopic, std::bind(&Echo::callback, this, std::placeholders::_1), nh);
    subscriber_ = cosmo_ros::subscribeShmRos<Msg_, MsgRos_, Converter_>(subOptions);
  }

  virtual ~Echo() = default;

  /*!
   * Blocking call. Registers a system signal handler and waits for incoming messages to republish.
   */
  void run()
  {
    signal_handler::SignalHandler::bindAll(&Echo::signalHandler, this);

    running_ = true;
    while (running_)
    {
      subscriber_->receive(std::chrono::microseconds{ 200000 });
    }

    signal_handler::SignalHandler::unbindAll(&Echo::signalHandler, this);
  }

private:
  void callback(const Msg_& msg)
  {
    std::cout << msg << std::endl;
  }

  void signalHandler(int)
  {
    running_ = false;
  }

public:
  std::atomic<bool> running_;
  SubscriberImpl subscriber_;
};

}  // namespace
