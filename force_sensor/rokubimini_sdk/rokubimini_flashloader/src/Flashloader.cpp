#include <sstream>
#include <iomanip>
#include <soem/oshw/nicdrv.h>
#include "rokubimini_flashloader/Flashloader.h"

#define OD_FIRMWARE_VERSION (0x7070)
#define OD_BOOTLOADER_VERSION (0x7071)
#define OD_SERIAL_NUMBER (0x7072)
#define OD_FIRMWARE_INFO (0x7075)

namespace rokubimini_flashloader
{
/* ========================================================================== */
/* Constructor                                                     */
/* ========================================================================== */

Flashloader::Flashloader()
{
  // Initialize EtherCAT context.
  ecatContext_.port->redport = nullptr;
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

bool Flashloader::isConnected()
{
  return isConnected_;
}

void Flashloader::setInterface(const std::string& interface)
{
  interface_ = interface;
}

void Flashloader::setSlave(const uint16_t& slave)
{
  slave_ = slave;
}

void Flashloader::setFilePath(const std::string& filePath)
{
  filePath_ = filePath;
}

void Flashloader::setBinaryName(const std::string& binaryName)
{
  binaryName_ = binaryName;
}

bool Flashloader::connect(std::string& message)
{
  isConnected_ = false;

  // Open interface.
  if (!openInterface(message))
  {
    return false;
  }

  // Connect slave.
  isConnected_ = connectSlave(message);

  return isConnected_;
}

bool Flashloader::openInterface(std::string& message)
{
  if (interface_.empty())
  {
    if (print_)
    {
      std::cout << "Set interface first." << std::endl;
    }
    isConnected_ = false;
    return false;
  }

  // Initialize interface.
  if (ecx_init(&ecatContext_, interface_.c_str()) <= 0)
  {
    message = "Failed to open socket. Execute as root.";
    if (print_)
    {
      std::cout << message << std::endl;
    }
    isConnected_ = false;
    return false;
  }

  if (print_)
  {
    std::cout << "Opened interface '" << interface_ << "' successfully." << std::endl;
  }

  return true;
}

bool Flashloader::connectSlave(std::string& message)
{
  if (slave_ == 0)
  {
    if (print_)
    {
      std::cout << "Set slave first." << std::endl;
    }
    return false;
  }

  if (ecx_config_init(&ecatContext_, FALSE) <= 0)
  {
    message = "No slaves found.";
    if (print_)
    {
      std::cout << message << std::endl;
    }
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }

  message = "Found " + std::to_string(ecatSlavecount_) + " slave(s).";
  if (print_)
  {
    std::cout << message << std::endl;
  }

  if (slave_ > ecatSlavecount_)
  {
    message = "Could not find slave " + std::to_string(slave_) + ".";
    if (print_)
    {
      std::cout << message << std::endl;
    }
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }

  return true;
}

bool Flashloader::switchToBootState()
{
  // Set slave to EC_STATE_INIT state.
  ecatSlavelist_[slave_].state = EC_STATE_INIT;
  ecx_writestate(&ecatContext_, slave_);
  if (ecx_statecheck(&ecatContext_, slave_, EC_STATE_INIT, EC_TIMEOUTSTATE * 4) != EC_STATE_INIT)
  {
    if (print_)
    {
      std::cout << "Could not set slave '" << slave_ << "' to EC_STATE_INIT state." << std::endl;
    }
    printEcatErrors();
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }

  // Set slave to EC_STATE_BOOT state.
  ecatSlavelist_[slave_].state = EC_STATE_BOOT;
  ecx_writestate(&ecatContext_, slave_);
  if (ecx_statecheck(&ecatContext_, slave_, EC_STATE_BOOT, EC_TIMEOUTSTATE * 10) != EC_STATE_BOOT)
  {
    if (print_)
    {
      std::cout << "Could not set slave '" << slave_ << "' to EC_STATE_BOOT state." << std::endl;
    }
    printEcatErrors();
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }

  return true;
}

bool Flashloader::readFileToBuffer()
{
  // Read file.
  fileBuffer_ = new char[MAX_FILE_SIZE];
  if (!readFile(filePath_, fileBuffer_, fileSize_))
  {
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }
  std::cout << "fileBuffer_: " << fileBuffer_ << std::endl;
  return true;
}

bool Flashloader::flash(std::string& message)
{
  message = "";

  // Send file over EtherCAT.
  if (print_)
  {
    std::cout << "Flashing ..." << std::endl;
  }
  std::string filenameTrunk = "";
  std::string imageType = "";
  int maxFileSize = 0;
  // Check file name.
  try
  {
    if (binaryName_.empty())
    {
      std::string filename = filePath_.substr(filePath_.find_last_of("/") + 1);
      if (containsString(filename, "rokubimini_firmware") || containsString(filename, "firmware"))
      {
        filenameTrunk = "firmware";
      }
      else if (containsString(filename, "rokubimini_bootloader") || containsString(filename, "bootloader"))
      {
        filenameTrunk = "bootloader";
      }
    }
    else
    {
      filenameTrunk = binaryName_;
    }
    imageType = "ecat_appl_ti";
    maxFileSize = MAX_FILE_SIZE_FIRMWARE;

    // if (filenameTrunk.compare("firmware") == 0) {
    //   imageType = "ecat_appl_ti";
    //   maxFileSize = MAX_FILE_SIZE_FIRMWARE;
    // } else if (filenameTrunk.compare("bootloader") == 0) {
    //   imageType = "bootloader";
    //   maxFileSize = MAX_FILE_SIZE_BOOTLOADER;
    // }
  }
  catch (const std::exception& exception)
  {
    message = "Caught out of range exception while finding filename trunk: " + std::string(exception.what());
    if (print_)
    {
      std::cout << message << std::endl;
    }
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }
  // Check file size.
  if (fileSize_ > maxFileSize)
  {
    message = "File size is too large (" + std::to_string(fileSize_) + " > " + std::to_string(maxFileSize) + ").";
    if (print_)
    {
      std::cout << message << std::endl;
    }
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }
  // Write over EtherCAT.
  uint32_t password = 0;
  std::cout << "Writing file over ethercat" << std::endl;
  std::cout << "fileSize_: " << fileSize_ << std::endl;

  int wkc = ecx_FOEwrite(&ecatContext_, slave_, &imageType[0], password, fileSize_, fileBuffer_, EC_TIMEOUTSTATE * 10);
  // Check work counter.
  if (wkc != 1)
  {
    message = "Writing file over EtherCAT failed (wkc = " + std::to_string(wkc) + ").";
    if (print_)
    {
      std::cout << message << std::endl;
    }
    isConnected_ = false;
    ecx_close(&ecatContext_);
    return false;
  }

  // Set slave to EC_STATE_INIT state.
  ecatSlavelist_[slave_].state = EC_STATE_INIT;
  ecx_writestate(&ecatContext_, slave_);

  message = "Flashing over EtherCAT was successful.";
  if (print_)
  {
    std::cout << message << std::endl;
  }

  binaryName_ = "";

  return true;
}

bool Flashloader::closeInterface()
{
  // Close socket.
  isConnected_ = false;
  ecx_close(&ecatContext_);

  return true;
}

int Flashloader::getSlaveCount()
{
  return ecatSlavecount_;
}

void Flashloader::getVersion(std::string& firmware, std::string& bootloader)
{
  firmware = getString(slave_, OD_FIRMWARE_VERSION);
  bootloader = getString(slave_, OD_BOOTLOADER_VERSION);
}

void Flashloader::getSerialNumber(uint16_t slaveId, std::string& serialNumber)
{
  serialNumber = getString(slaveId, OD_SERIAL_NUMBER);
}

bool Flashloader::getFirmwareInfo(uint16_t slaveId)
{
  firmwareInfo_.hasInfo = false;
  if (!getFirmwareInfo(slaveId, OD_FIRMWARE_INFO))
  {
    if (print_)
    {
      std::cout << "Could not get firmware info!" << std::endl;
    }
    return false;
  }

  firmwareInfo_.infoVersion = firmwareInfoRaw_[0];

  if (firmwareInfo_.infoVersion == 3)
  {
    // Get version.
    std::stringstream versionStream;
    for (int i = 0; i < 3; ++i)
    {
      versionStream << (int)firmwareInfoRaw_[1 + i];
      if (i < 2)
      {
        versionStream << ".";
      }
    }
    firmwareInfo_.version = versionStream.str();

    // Get fw hash.
    std::stringstream hashStream;
    for (int i = 0; i < 16; ++i)
    {
      if (firmwareInfoRaw_[4 + i] == '\0')
      {
        break;
      }
      hashStream << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex << (int)firmwareInfoRaw_[4 + i];
    }
    firmwareInfo_.fwHash = hashStream.str();

    // Get channel id.
    std::stringstream channelIdStream;
    for (int i = 0; i < 16; ++i)
    {
      if (firmwareInfoRaw_[20 + i] == '\0')
      {
        break;
      }
      channelIdStream << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex
                      << (int)firmwareInfoRaw_[20 + i];
    }
    firmwareInfo_.channelId = channelIdStream.str();

    // Get channel id.
    std::stringstream channelTidStream;
    for (int i = 0; i < 16; ++i)
    {
      if (firmwareInfoRaw_[36 + i] == '\0')
      {
        break;
      }
      channelTidStream << firmwareInfoRaw_[36 + i];
    }
    firmwareInfo_.channelTid = channelTidStream.str();

    // Get serial number.
    std::stringstream serialNumberStream;
    for (int i = 0; i < 16; ++i)
    {
      if (firmwareInfoRaw_[52 + i] == '\0')
      {
        break;
      }
      serialNumberStream << firmwareInfoRaw_[52 + i];
    }
    firmwareInfo_.serialNumber = serialNumberStream.str();

    // Get key id.
    std::stringstream keyIdStream;
    for (int i = 0; i < 16; ++i)
    {
      keyIdStream << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex << (int)firmwareInfoRaw_[68 + i];
      if (i == 3 || i == 5 || i == 7 || i == 9)
      {
        keyIdStream << "-";
      }
    }
    firmwareInfo_.keyId = keyIdStream.str();

    // Print result.
    if (print_)
    {
      std::cout << "Firmware info version:  " << (int)firmwareInfo_.infoVersion << std::endl;
      std::cout << "Firmware version:       " << firmwareInfo_.version << std::endl;
      std::cout << "Firmware hash:          " << firmwareInfo_.fwHash << std::endl;
      std::cout << "Firmware channel id:    " << firmwareInfo_.channelId << std::endl;
      std::cout << "Firmware channel tid:   " << firmwareInfo_.channelTid << std::endl;
      std::cout << "Firmware serial number: " << firmwareInfo_.serialNumber << std::endl;
      std::cout << "Firmware key id:        " << firmwareInfo_.keyId << std::endl;
    }
  }
  else
  {
    if (print_)
    {
      std::cout << "Firmware info version (" << (int)firmwareInfo_.infoVersion
                << ") is not compatible with this sofware version. "
                   "Please update the software."
                << std::endl;
    }
    return false;
  }

  firmwareInfo_.hasInfo = true;
  return true;
}

firmwareInfo_t Flashloader::getFirmwareInfo()
{
  return firmwareInfo_;
}

void Flashloader::enablePrint(bool enable)
{
  print_ = enable;
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

bool Flashloader::readFile(const std::string& filepath, char* filebuffer, int& filesize)
{
  if (print_)
  {
    std::cout << "Reading file ..." << filebuffer << std::endl;
  }

  FILE* file;
  int counter = 0;
  int character = 0;

  file = fopen(filepath.c_str(), "rb");

  if (file == nullptr)
  {
    if (print_)
    {
      std::cout << "Could not open file." << std::endl;
    }
    return false;
  }

  while ((character = fgetc(file)) != EOF && counter < MAX_FILE_SIZE)
    filebuffer[counter++] = static_cast<uint8>(character);

  if (counter == MAX_FILE_SIZE)
  {
    if (print_)
    {
      std::cout << "File is too big." << std::endl;
    }
    fclose(file);
    return false;
  }

  filesize = counter;
  fclose(file);

  if (print_)
  {
    std::cout << "Read " << filesize << " bytes from file." << std::endl;
  }
  return true;
}

// bool Flashloader::readFile(const std::string &filepath, unsigned char *filebuffer,
//                            int &filesize) {

//   std::ifstream input(filepath, std::ios::binary);
//   int counter = 0;
//   while(input) {
//     std::ios::pos_type before = input.tellg();
//     input >> filebuffer[counter];
//     unsigned char x;
//     input >> x;
//     std::ios::pos_type after = input.tellg();
//     std::cout << before << ' ' << static_cast<int>(filebuffer[counter]) << ' '
//               << after << std::endl;
//     ++counter;
//   }
//   // std::vector<char> buffer((
//   //         std::istreambuf_iterator<char>(input)),
//   //         (std::istreambuf_iterator<char>()));
//   // std::cout << "buffer size: " << buffer.size() << std::endl;

//   // std::cout << "buffer: " << std::endl;
//   // for(int i = 0; i < buffer.size(); ++i) {
//   //   std::cout << std::hex << buffer[i] << std::endl;
//   // }

//   // filebuffer = &buffer[0];
//   filesize = counter;

// }

void Flashloader::printEcatErrors()
{
  if (print_)
  {
    std::cout << "EtherCAT errors:" << std::endl;
  }
  for (int i = 1; i <= *ecatContext_.slavecount; i++)
  {
    printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ecatContext_.slavelist[i].state,
           ecatContext_.slavelist[i].ALstatuscode, ec_ALstatuscode2string(ecatContext_.slavelist[i].ALstatuscode));
  }
}

std::string Flashloader::getString(uint16_t slaveId, uint16_t index)
{
  const uint8_t lenSubindex = 0;
  const uint8_t dataSubindex = 1;

  if (slaveId > ecatSlavecount_)
  {
    if (print_)
    {
      std::cout << "Slave ID (" << slaveId << ") is not valid!" << std::endl;
    }
    return "";
  }

  // Read number of subindices.
  uint8_t nSubindex = 0;
  int size = sizeof(uint8_t);
  int wkc_ = ecx_SDOread(&ecatContext_, slaveId, index, lenSubindex, false, &size, &nSubindex, EC_TIMEOUTRXM);

  // Read char casted as an uint32 array.
  const uint8_t nChars = nSubindex * sizeof(uint32_t) / sizeof(char);
  char arrayChar[nChars + 1];
  size = sizeof(uint32_t);
  for (uint8_t i = 0; i < nSubindex; i++)
  {
    wkc_ = ecx_SDOread(&ecatContext_, slaveId, index, dataSubindex + i, false, &size, &((uint32_t*)arrayChar)[i],
                       EC_TIMEOUTRXM);
  }
  arrayChar[nChars] = '\0';

  // Transform char array to string.
  std::string str = std::string(arrayChar);

  return str;
}

bool Flashloader::getFirmwareInfo(uint16_t slaveId, uint16_t index)
{
  const uint8_t lenSubindex = 0;
  const uint8_t dataSubindex = 1;

  if (slaveId > ecatSlavecount_)
  {
    if (print_)
    {
      std::cout << "Slave ID (" << slaveId << ") is not valid!" << std::endl;
    }
    return false;
  }

  // Read number of subindices.
  uint8_t nSubindex = 0;
  int size = sizeof(uint8_t);
  int wkc_ = ecx_SDOread(&ecatContext_, slaveId, index, lenSubindex, false, &size, &nSubindex, EC_TIMEOUTRXM);
  if (wkc_ != 1)
  {
    if (print_)
    {
      std::cout << "Working counter (" << wkc_ << ") too low." << std::endl;
    }
    return false;
  }

  // Read char casted as an uint32 array.
  const uint8_t nChars = nSubindex * sizeof(uint32_t) / sizeof(char);
  if (nChars != 128)
  {
    if (print_)
    {
      std::cout << "Firmware info array has the wrong length (" << (int)nChars << ")!" << std::endl;
    }
    return false;
  }
  size = sizeof(uint32_t);
  for (uint8_t i = 0; i < nSubindex; i++)
  {
    wkc_ = ecx_SDOread(&ecatContext_, slaveId, index, dataSubindex + i, false, &size, &((uint32_t*)firmwareInfoRaw_)[i],
                       EC_TIMEOUTRXM);
    if (wkc_ != 1)
    {
      if (print_)
      {
        std::cout << "Working counter (" << wkc_ << ") too low." << std::endl;
      }
      return false;
    }
  }

  return true;
}

}  // namespace
