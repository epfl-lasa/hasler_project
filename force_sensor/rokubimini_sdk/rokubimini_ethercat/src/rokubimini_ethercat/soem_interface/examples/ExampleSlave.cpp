#include <rokubimini_ethercat/soem_interface/examples/ExampleSlave.hpp>

namespace soem_interface_examples
{
ExampleSlave::ExampleSlave(const std::string& name, soem_interface::EthercatBusBase* bus, const uint32_t address)
  : soem_interface::EthercatSlaveBase(bus, address), name_(name)
{
  pdoInfo_.rxPdoId_ = RX_PDO_ID;
  pdoInfo_.txPdoId_ = TX_PDO_ID;
  pdoInfo_.rxPdoSize_ = sizeof(command_);
  pdoInfo_.txPdoSize_ = sizeof(reading_);
  pdoInfo_.moduleId_ = 0x00123456;
}

bool ExampleSlave::startup()
{
  // Do nothing else
  return true;
}

void ExampleSlave::updateRead()
{
  bus_->readTxPdo(address_, reading_);
}

void ExampleSlave::updateWrite()
{
  bus_->writeRxPdo(address_, command_);
}

void ExampleSlave::shutdown()
{
  // Do nothing
}
}  // namespace soem_interface_examples