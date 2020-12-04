/*!
 * @file	Relay.hpp
 * @author	Philipp Leemann
 * @date	Jan, 2018
 */

#pragma once

#include "cosmo_ros/cosmo_ros.hpp"
#include "signal_handler/SignalHandler.hpp"

#include <atomic>

namespace cosmo_ros_tools
{
template <typename Msg_, typename MsgRos_, template <typename, typename> class Converter_ = cosmo_ros::ConversionTraits>
class Relay
{
public:
  using SubscriberImpl = cosmo_ros::SubscriberRosPtr<Msg_, MsgRos_, Converter_>;
  using PublisherImpl = cosmo_ros::PublisherRosPtr<Msg_, MsgRos_, Converter_>;

  Relay() = delete;

  /*!
   * @param inTopic   Name of the topic to listen to
   * @param outTopic  Name of the topic to republish on
   */
  Relay(int argc, char** argv, const std::string& inTopic, const std::string& outTopic) : running_{ false }
  {
    ros::init(argc, argv, "Relay", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    auto subOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<Msg_>>(
        inTopic, std::bind(&Relay::callback, this, std::placeholders::_1), nh);
    auto pubOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(outTopic, nh);
    subscriber_ = cosmo_ros::subscribeShmRos<Msg_, MsgRos_, Converter_>(subOptions);
    publisher_ = cosmo_ros::advertiseShmRos<Msg_, MsgRos_, Converter_>(pubOptions);
  }

  virtual ~Relay() = default;

  /*!
   * Blocking call. Registers a system signal handler and waits for incoming messages to republish.
   */
  void run()
  {
    signal_handler::SignalHandler::bindAll(&Relay::signalHandler, this);

    running_ = true;
    while (running_)
    {
      subscriber_->receive(std::chrono::microseconds{ 200000 });
    }

    signal_handler::SignalHandler::unbindAll(&Relay::signalHandler, this);
  }

private:
  void callback(const Msg_& msg)
  {
    publisher_->publishAndSend(msg);
  }

  void signalHandler(int)
  {
    running_ = false;
  }

public:
  std::atomic<bool> running_;
  SubscriberImpl subscriber_;
  PublisherImpl publisher_;
};

}  // namespace
