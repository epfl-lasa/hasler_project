#pragma once

#include <rokubimini/Command.hpp>
#include <rokubimini/Reading.hpp>
#include <rokubimini/Statusword.hpp>

#include <rokubimini_msgs/Command.h>
#include <rokubimini_msgs/Reading.h>

#include <any_measurements_ros/ConvertRosMessages.hpp>

#include <kindr_ros/kindr_ros.hpp>

namespace rokubimini_ros
{
template <typename Msg_, typename MsgRos_>
class ConversionTraits;

template <>
class ConversionTraits<rokubimini::Reading, rokubimini_msgs::Reading>
{
public:
  using Msg = rokubimini::Reading;
  using MsgRos = rokubimini_msgs::Reading;

  inline static void convert(const Msg& msg, MsgRos& rosMsg)
  {
    rosMsg.header.stamp = any_measurements_ros::toRos(msg.getWrench().time_);
    rosMsg.imu = any_measurements_ros::toRos(msg.getImu());
    rosMsg.wrench = any_measurements_ros::toRos(msg.getWrench());
    rosMsg.externalImu = any_measurements_ros::toRos(msg.getExternalImu());
    rosMsg.isForceTorqueSaturated = msg.isForceTorqueSaturated();
    rosMsg.statusword = msg.getStatusword().getData();
    rosMsg.temperature = msg.getTemperature();
  }

  inline static void convert(const MsgRos& rosMsg, Msg& msg)
  {
    msg.getWrench().time_ = any_measurements_ros::fromRos(rosMsg.header.stamp);
    msg.getImu() = any_measurements_ros::fromRos(rosMsg.imu);
    msg.getWrench() = any_measurements_ros::fromRos(rosMsg.wrench);
    msg.getExternalImu() = any_measurements_ros::fromRos(rosMsg.externalImu);
    msg.setForceTorqueSaturated(rosMsg.isForceTorqueSaturated);
    msg.setStatusword(rokubimini::Statusword(rosMsg.statusword));
    msg.setTemperature(rosMsg.temperature);
  }

  inline static MsgRos convert(const Msg& msg)
  {
    MsgRos ros_msg;
    convert(msg, ros_msg);
    return ros_msg;
  }

  inline static Msg convert(const MsgRos& rosMsg)
  {
    Msg msg;
    convert(rosMsg, msg);
    return msg;
  }
};

template <>
class ConversionTraits<rokubimini::Command, rokubimini_msgs::Command>
{
public:
  using Msg = rokubimini::Command;
  using MsgRos = rokubimini_msgs::Command;

  inline static void convert(const Msg& msg, MsgRos& rosMsg)
  {
    rosMsg.header.stamp = any_measurements_ros::toRos(msg.getStamp());
    rosMsg.setAsZeroTare = msg.isSetAsZeroTare();
    rosMsg.resetTareLoad = msg.isResetTareLoad();
    rosMsg.zeroTare = any_measurements_ros::toRos(msg.getZeroTare());
  }

  inline static void convert(const MsgRos& rosMsg, Msg& msg)
  {
    msg.setStamp(any_measurements_ros::fromRos(rosMsg.header.stamp));
    msg.setAsZeroTare(rosMsg.setAsZeroTare);
    msg.setResetTareLoad(rosMsg.resetTareLoad);
    msg.getZeroTare() = any_measurements_ros::fromRos(rosMsg.zeroTare);
  }

  inline static MsgRos convert(const Msg& msg)
  {
    MsgRos ros_msg;
    convert(msg, ros_msg);
    return ros_msg;
  }

  inline static Msg convert(const MsgRos& rosMsg)
  {
    Msg msg;
    convert(rosMsg, msg);
    return msg;
  }
};

}  // namespace rokubimini_ros