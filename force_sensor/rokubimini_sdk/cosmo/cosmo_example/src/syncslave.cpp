/*
 * syncmaster.cpp
 *
 *  Created on: Jul 2, 2017
 *      Author: gehrinch
 */

#ifndef COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL1
#define COSMO_DEBUG_LEVEL2
#endif

#include "cosmo/cosmo.hpp"
#include "cosmo/SyncSlave.hpp"

#include "message_logger/message_logger.hpp"

#include <sys/time.h>
#include <signal.h>

bool g_running = true;

void signalCallback(int)
{
  g_running = false;
}

int main()
{
  signal(SIGINT, signalCallback);
  cosmo::SyncSlave sync("test_sync");
  sync.start();

  while (g_running)
  {
    auto msg = sync.waitForSync();
    MELO_INFO_STREAM("Start working with sync: " << msg);
    std::this_thread::sleep_for(std::chrono::milliseconds{ 1100 });
    MELO_INFO_STREAM("End working with sync: " << msg);
  }

  return true;
}
