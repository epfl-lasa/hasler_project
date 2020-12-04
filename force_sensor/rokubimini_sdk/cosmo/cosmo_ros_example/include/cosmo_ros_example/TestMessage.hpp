/*!
 * @file	TestMessage.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	 25.05.17.
 */
#pragma once

#include "cosmo_ros_example/TestMessage.h"

namespace shared_memory
{
//! Message to store in shared memory
class TestMessage
{
public:
  TestMessage() : a(0)
  {
  }

  int a;
};
}

namespace cosmo_ros_example
{
// Forward declaration
template <typename Msg_, typename MsgRos_>
class ConversionTraits;

//! Conversions between shared memory message and ROS message.
template <>
class ConversionTraits<shared_memory::TestMessage, cosmo_ros_example::TestMessage>
{
public:
  inline static cosmo_ros_example::TestMessage convert(const shared_memory::TestMessage& shmMsg)
  {
    cosmo_ros_example::TestMessage rosMsg;
    rosMsg.a = shmMsg.a;
    return rosMsg;
  }

  inline static shared_memory::TestMessage convert(const cosmo_ros_example::TestMessage& rosMsg)
  {
    shared_memory::TestMessage shmMsg;
    shmMsg.a = rosMsg.a;
    return shmMsg;
  }
};

}  // namespace
