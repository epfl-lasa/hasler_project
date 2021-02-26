#pragma once

#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <string>

#include <soem/soem/ethercat.h>

#define MAX_FILE_SIZE (0xF00000)
#define MAX_FILE_SIZE_BOOTLOADER (0x07F000)
#define MAX_FILE_SIZE_FIRMWARE (0x100000)

namespace rokubimini_flashloader
{
struct firmwareInfo_t
{
  bool hasInfo = false;
  uint8_t infoVersion = 0;
  std::string version = "";
  std::string fwHash = "";
  std::string channelId = "";
  std::string channelTid = "";
  std::string serialNumber = "";
  std::string keyId = "";
};

class Flashloader
{
public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  Flashloader();

  virtual ~Flashloader() = default;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  bool isConnected();

  void setInterface(const std::string& interface);

  void setSlave(const uint16_t& slave);

  void setFilePath(const std::string& filePath);

  void setBinaryName(const std::string& binaryName);

  bool connect(std::string& message);

  bool switchToBootState();

  bool readFileToBuffer();

  bool flash(std::string& message);

  bool closeInterface();

  int getSlaveCount();

  void getVersion(std::string& firmware, std::string& bootloader);

  void getSerialNumber(uint16_t slaveId, std::string& serialNumber);

  bool getFirmwareInfo(uint16_t slaveId);

  firmwareInfo_t getFirmwareInfo();

  void enablePrint(bool enable);

private:
  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  bool print_ = false;

  std::string interface_ = "";
  std::string filePath_ = "";
  uint16_t slave_ = 0;
  std::string binaryName_ = "";

  uint8_t firmwareInfoRaw_[128];
  firmwareInfo_t firmwareInfo_;

  // EtherCAT context data elements:

  // Port reference.
  ecx_portt ecatPort_;
  // List of slave data. Index 0 is reserved for the master, higher indices for the slaves.
  ec_slavet ecatSlavelist_[EC_MAXSLAVE];
  // Number of slaves found in the network.
  int ecatSlavecount_ = 0;
  // Slave group structure.
  ec_groupt ecatGrouplist_[EC_MAXGROUP];
  // Internal, reference to EEPROM cache buffer.
  uint8 ecatEsiBuf_[EC_MAXEEPBUF];
  // Internal, reference to EEPROM cache map.
  uint32 ecatEsiMap_[EC_MAXEEPBITMAP];
  // Internal, reference to error list.
  ec_eringt ecatEList_;
  // Internal, reference to processdata stack buffer info.
  ec_idxstackT ecatIdxStack_;
  // Boolean indicating if an error is available in error stack.
  boolean ecatError_ = FALSE;
  // Reference to last DC time from slaves.
  int64 ecatDcTime_ = 0;
  // Internal, SM buffer.
  ec_SMcommtypet ecatSmCommtype_[EC_MAX_MAPT];
  // Internal, PDO assign list.
  ec_PDOassignt ecatPdoAssign_[EC_MAX_MAPT];
  // Internal, PDO description list.
  ec_PDOdesct ecatPdoDesc_[EC_MAX_MAPT];
  // Internal, SM list from EEPROM.
  ec_eepromSMt ecatSm_;
  // Internal, FMMU list from EEPROM.
  ec_eepromFMMUt ecatFmmu_;

  // EtherCAT context data.
  ecx_contextt ecatContext_ = { &ecatPort_,
                                &ecatSlavelist_[0],
                                &ecatSlavecount_,
                                EC_MAXSLAVE,
                                &ecatGrouplist_[0],
                                EC_MAXGROUP,
                                &ecatEsiBuf_[0],
                                &ecatEsiMap_[0],
                                0,
                                &ecatEList_,
                                &ecatIdxStack_,
                                &ecatError_,
                                0,
                                0,
                                &ecatDcTime_,
                                &ecatSmCommtype_[0],
                                &ecatPdoAssign_[0],
                                &ecatPdoDesc_[0],
                                &ecatSm_,
                                &ecatFmmu_,
                                nullptr };

  char* fileBuffer_ = nullptr;
  int fileSize_ = 0;

  bool isConnected_ = false;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  bool readFile(const std::string& filepath, char* filebuffer, int& filesize);

  void printEcatErrors();

  bool openInterface(std::string& message);

  bool connectSlave(std::string& message);

  std::string getString(uint16_t slaveId, uint16_t index);

  bool containsString(std::string str, std::string subStr)
  {
    return str.find(subStr) != std::string::npos;
  }

  bool getFirmwareInfo(uint16_t slaveId, uint16_t index);
};

}  // namespace
