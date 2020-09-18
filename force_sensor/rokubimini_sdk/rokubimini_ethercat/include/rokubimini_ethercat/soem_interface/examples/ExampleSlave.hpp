#pragma once

#include <rokubimini_ethercat/soem_interface/EthercatBusBase.hpp>
#include <rokubimini_ethercat/soem_interface/EthercatSlaveBase.hpp>

#define RX_PDO_ID 0x6000
#define TX_PDO_ID 0x7000

namespace soem_interface_examples
{
struct TxPdo
{
  uint8_t state = 0;
  float data1 = 0.0;
  float data2 = 0.0;
} __attribute__((packed));

struct RxPdo
{
  float command1 = 0.0;
  float command2 = 0.0;
} __attribute__((packed));

class ExampleSlave : public soem_interface::EthercatSlaveBase
{
public:
  ExampleSlave(const std::string& name, soem_interface::EthercatBusBase* bus, const uint32_t address);
  ~ExampleSlave() override = default;

  std::string getName() const override
  {
    return name_;
  }

  bool startup() override;
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;

  PdoInfo getCurrentPdoInfo() const override
  {
    return pdoInfo_;
  }

private:
  const std::string name_;
  PdoInfo pdoInfo_;
  TxPdo reading_;
  RxPdo command_;
};
}  // namespace soem_interface_examples