//
// Created by pleemann on 25.05.17.
//

#ifndef COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL2
#endif

#include "cosmo_ros/cosmo_ros.hpp"
#include "cosmo_ros_example/TestMessage.hpp"

#include <csignal>
#include <cstdlib>
#include <ctime>
#include <atomic>

std::atomic<bool> g_running{ true };

void signalCallback(int)
{
  g_running = false;
}

class MyClass
{
public:
  void callback(const shared_memory::TestMessage& msg)
  {
    MELO_INFO_STREAM("Subscriber received a: " << msg.a);
  }
};

int main(int argc, char** argv)
{
  signal(SIGINT, signalCallback);

  std::srand(std::time(nullptr));  // use current time as seed for random generator
  ros::init(argc, argv, "listener" + std::to_string(std::rand()), ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  MyClass instance;

  auto options = std::make_shared<cosmo_ros::SubscriberRosOptions<shared_memory::TestMessage>>(
      "/test", std::bind(&MyClass::callback, &instance, std::placeholders::_1), nh);
  options->autoSubscribe_ = true;
  options->tryRosResubscribing_ = true;
  auto sub = cosmo_ros::subscribeShmRos<shared_memory::TestMessage, cosmo_ros_example::TestMessage,
                                        cosmo_ros_example::ConversionTraits>(options);

  while (g_running)
  {
    if (!options->autoSubscribe_)
    {
      sub->receive(std::chrono::microseconds{ 1000000 });
    }
    else
    {
      sleep(1);
    }
  }

  return true;
}
