#include <rokubimini_ethercat/soem_interface/examples/ExampleSlave.hpp>
#include <rokubimini_ethercat/soem_interface/EthercatBusBase.hpp>

// This shows a minimal example on how to use the soem_interface library.
// Keep in mind that this is non-working example code, with only minimal error handling

int main(int argc, char** argv)
{
  const std::string bus_name = "eth1";
  const std::string slave_name = "ExampleSlave";
  const uint32_t slave_address = 0;

  std::unique_ptr<soem_interface::EthercatBusBase> bus = std::make_unique<soem_interface::EthercatBusBase>(bus_name);

  std::shared_ptr<soem_interface_examples::ExampleSlave> slave =
      std::make_shared<soem_interface_examples::ExampleSlave>(slave_name, bus.get(), slave_address);

  bus->addSlave(slave);
  bus->startup();
  bus->setState(EC_STATE_OPERATIONAL);

  if (!bus->waitForState(EC_STATE_OPERATIONAL, slave_address))
  {
    // Something is wrong
    return 1;
  }

  while (true)
  {
    bus->updateRead();
    bus->updateWrite();
  }

  bus->shutdown();
  return 0;
}