/*!
 * @file     CommunicatorNodeExample.cpp
 * @author   Philipp Leemann
 * @date     May, 2017
 * @brief
 */

#ifndef COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL2
#endif

#include "cosmo_node_example/CommunicatorNodeExample.hpp"
#include "cosmo_ros_example/TestMessage.hpp"

#include "message_logger/message_logger.hpp"

namespace cosmo_node_example
{
CommunicatorNodeExample::CommunicatorNodeExample(NodeHandlePtr nh) : cosmo_node::Node(nh), pub_(), sub_(), msg_()
{
}

CommunicatorNodeExample::~CommunicatorNodeExample()
{
}

bool CommunicatorNodeExample::init()
{
  using namespace cosmo_ros_example;
  MELO_INFO("Init");

  auto pubOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("/test");
  pub_ = advertiseShmRos<shared_memory::TestMessage, TestMessage, ConversionTraits>(pubOptions);

  auto subOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<shared_memory::TestMessage>>(
      "/test", std::bind(&CommunicatorNodeExample::callback, this, std::placeholders::_1));
  subOptions->autoSubscribe_ = true;
  sub_ = subscribeShmRos<shared_memory::TestMessage, TestMessage, ConversionTraits>(subOptions);

  addWorker("CommunicatorNodeExample::updateWorker", param<double>("time_step", 1.0), &CommunicatorNodeExample::update,
            this);

  return true;
}

void CommunicatorNodeExample::cleanup()
{
  MELO_INFO("Cleanup.");
  // stopping of subscriber required here if autoSubscribe_ is set to true
  sub_->stop(true);
}

bool CommunicatorNodeExample::update(const any_worker::WorkerEvent& event)
{
  MELO_INFO("Update.");

  ++(msg_.a);
  pub_->publish(msg_);
  pub_->sendRos();

  return true;
}

void CommunicatorNodeExample::callback(const shared_memory::TestMessage& msg)
{
  std::cout << msg.a << std::endl;
}
}
