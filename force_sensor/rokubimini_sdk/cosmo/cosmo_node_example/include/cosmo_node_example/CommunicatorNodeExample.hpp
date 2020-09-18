/*!
 * @file     CommunicatorNodeExample.hpp
 * @author   Philipp Leemann
 * @date     May, 2017
 * @brief
 */

#pragma once

#include "cosmo_ros_example/TestMessage.hpp"

#include "cosmo_node/Node.hpp"

namespace cosmo_node_example
{
class CommunicatorNodeExample : public cosmo_node::Node
{
public:
  // constructor/destructor
  CommunicatorNodeExample() = delete;
  CommunicatorNodeExample(NodeHandlePtr nh);
  virtual ~CommunicatorNodeExample();

  // virtual functions from NodeImpl
  bool init() override;
  void cleanup() override;

  bool update(const any_worker::WorkerEvent& event);

  void callback(const shared_memory::TestMessage& msg);

protected:
  cosmo_ros::PublisherRosPtr<shared_memory::TestMessage, cosmo_ros_example::TestMessage,
                             cosmo_ros_example::ConversionTraits>
      pub_;
  cosmo_ros::SubscriberRosPtr<shared_memory::TestMessage, cosmo_ros_example::TestMessage,
                              cosmo_ros_example::ConversionTraits>
      sub_;

  shared_memory::TestMessage msg_;
};
}
