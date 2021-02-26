//
// Created by pleemann on 25.05.17.
//

#ifndef COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL2
#endif

#include "cosmo/cosmo.hpp"

#include "message_logger/message_logger.hpp"

#include "cosmo_example/TestMessage.hpp"

#include <sys/time.h>
#include <signal.h>
#include <atomic>
#include <array>

std::atomic<bool> g_running{ true };

void signalCallback(int)
{
  g_running = false;
}

int main()
{
  signal(SIGINT, signalCallback);

  // create a couple of publishers. We will use the last publisher as "timer" in the subscribing program
  std::array<cosmo::PublisherPtr<TestMessage>, 5> publishers;

  // create 2 publishers using the helper function nr1
  publishers[0] = cosmo::advertiseShm<TestMessage>("/test0");
  publishers[1] = cosmo::advertiseShm<TestMessage>("/test1");

  // create publisher using helper function nr2
  {
    cosmo::PublisherOptionsPtr options(new cosmo::PublisherOptions("/test2"));
    publishers[2] = cosmo::advertiseShm<TestMessage>(options);
  }

  // we can also create publishers directly, using options, without helper function
  {
    cosmo::PublisherOptionsPtr options(new cosmo::PublisherOptions("/test3"));
    publishers[3].reset(new cosmo::Publisher<TestMessage>(options));
  }

  // we can also create publishers directly, without using options nor helper function
  publishers[4].reset(new cosmo::Publisher<TestMessage>("/test4"));

  // create and fill message
  TestMessage msg;
  for (unsigned int i = 0; i < msg.b.size(); ++i)
  {
    msg.b[i] = i + 3;
  }

  // main loop
  while (g_running)
  {
    ++msg.a;
    for (unsigned int i = 0; i < publishers.size(); ++i)
    {
      publishers[i]->publish(msg);
    }
    MELO_INFO("=========================================");
    sleep(1);
  }

  return true;
}
