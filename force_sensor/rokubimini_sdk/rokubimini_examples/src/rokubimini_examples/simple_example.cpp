
#include <unistd.h>
#include <string>

#include <rokubimini/Command.hpp>
#include <rokubimini/Reading.hpp>

#include <rokubimini_manager/Manager.hpp>

#include <message_logger/message_logger.hpp>

using namespace rokubimini;

int main(int argc, char** argv)
{
  constexpr static unsigned int waitMicroSeconds = 5000;
  const bool isStandalone = false;
  const bool installSignalHandler = false;

  std::string setupFile;
  if (argc == 2)
  {
    setupFile = argv[1];
  }
  else
  {
    MELO_WARN_STREAM("No config file specified");
    MELO_ERROR_STREAM(
        "Correct usage 'rosrun rokubimini_ethercat rokubimini_ethercat_ethercat_example path_to_setup_file'");
  }

  RokubiminiManager rokubiminiManager(isStandalone, installSignalHandler);
  rokubiminiManager.loadSetup(setupFile);
  rokubiminiManager.startup();

  auto rokubiminis = rokubiminiManager.getRokubiminis();

  const rokubimini::Command command;
  rokubimini::Reading reading;

  while (true)
  {
    rokubiminiManager.update();
    usleep(waitMicroSeconds);
  }

  rokubiminiManager.shutdown();
  return 0;
}