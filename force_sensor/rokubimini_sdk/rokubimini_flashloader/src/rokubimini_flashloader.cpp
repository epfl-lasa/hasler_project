#include "rokubimini_flashloader/Flashloader.h"

int main(int argc, char* argv[])
{
  std::cout << "Starting flashloader ..." << std::endl;

  // if (argc != 4) {
  //   std::cout
  //       << "Usage: sudo ./rokubimini_flashloader <interface_name> <slave_number>
  //       </path/to/rokubimini_firmware_vx.x.x.bin>"
  //       << std::endl;
  //   return 0;
  // }

  rokubimini_flashloader::Flashloader flashloader;
  flashloader.enablePrint(true);

  std::string interfaceName = "enxd8eb97bf8390";
  uint16_t slave = 1;
  std::string filePath = "/home/markusta/rokubimini/rokubimini_ethercat_firmware/build/xmc4800-ecat-slave.bin";
  // std::string filePath = "/home/markusta/git/rokubimini_ethercat_ros/rokubimini_flashloader/src/array.elf";

  flashloader.setInterface(interfaceName);
  flashloader.setSlave(slave);
  flashloader.setFilePath(filePath);

  // flashloader.setInterface(std::string(argv[1]));
  // flashloader.setSlave(atoi(argv[2]));
  // flashloader.setFilePath(std::string(argv[3]));

  std::string message;
  if (!flashloader.connect(message))
  {
    std::cout << "Could not open the interface." << std::endl;
    return -1;
  }

  if (!flashloader.switchToBootState())
  {
    std::cout << "Could not switch to boot state." << std::endl;
    return -2;
  }

  if (!flashloader.readFileToBuffer())
  {
    std::cout << "Could not read the file." << std::endl;
    return -3;
  }

  // flashloader.getFirmwareInfo(slave);

  if (!flashloader.flash(message))
  {
    std::cout << "Flashing was not successful." << std::endl;
    return -4;
  }

  if (!flashloader.closeInterface())
  {
    std::cout << "Could not close the interface." << std::endl;
    return -5;
  }

  std::cout << "Flashing was successful." << std::endl;
  return 0;
}
