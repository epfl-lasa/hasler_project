#include <rokubimini_ethercat/RokubiminiEthercatBusManager.hpp>

namespace rokubimini
{
namespace ethercat
{
bool RokubiminiEthercatBusManager::loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>>& rokubiminis)
{
  // Loop through all Rokubiminis and create their buses.
  for (const auto& rokubimini_setup : attachedRokubiminiSetups_)
  {
    const auto rokubimini_ethercat_setup = std::dynamic_pointer_cast<setup::RokubiminiEthercat>(rokubimini_setup);

    const std::string bus_name = rokubimini_ethercat_setup->ethercatBus_;
    if (bus_name.empty())
    {
      MELO_ERROR("[%s] The name of the bus is empty.", rokubimini_setup->name_.c_str());
      return false;
    }
    soem_interface::EthercatBusBase* bus;
    const auto& it = buses_.find(bus_name);
    if (it == buses_.end())
    {
      // Create a new bus.
      bus = new soem_interface::EthercatBusBase(bus_name);
      buses_.insert(std::make_pair(bus_name, std::unique_ptr<soem_interface::EthercatBusBase>(bus)));
    }
    else
    {
      // Take existing bus.
      bus = it->second.get();
    }

    for (const auto& rokubimini : rokubiminis)
    {
      if (rokubimini->getName() == rokubimini_ethercat_setup->name_)
      {
        auto rokubimini_ethercat = (RokubiminiEthercat*)(rokubimini.get());
        if (!addRokubiminiToBus(rokubimini_ethercat, bus, rokubimini_ethercat_setup))
        {
          return false;
        }
      }
    }
    // const auto rokubimini = dynamic_cast<RokubiminiEthercat *>(rokubiminis[i].get());
    // std::dynamic_pointer_cast<rokubimini::ethercat::RokubiminiEthercat>(rokubiminis[i]);
  }

  return true;
}

bool RokubiminiEthercatBusManager::addRokubiminiToBus(
    RokubiminiEthercat* rokubimini, soem_interface::EthercatBusBase* bus,
    const std::shared_ptr<setup::RokubiminiEthercat>& rokubiminiEthercatSetup) const
{
  auto slave = std::make_shared<RokubiminiEthercatSlave>(rokubiminiEthercatSetup->name_, bus,
                                                         rokubiminiEthercatSetup->ethercatAddress_,
                                                         rokubiminiEthercatSetup->ethercatPdoTypeEnum_);

  if (!bus->addSlave(slave))
  {
    return false;
  }

  rokubimini->setSlavePointer(slave);
  return true;
}

void RokubiminiEthercatBusManager::setConfigMode()
{
  setBussesPreOperational();
  // Sleep for some time to give the slave time to execute the pre-op cb
  soem_interface::threadSleep(0.5);
}
void RokubiminiEthercatBusManager::setRunMode()
{
  setBussesSafeOperational();
  setBussesOperational();
}
bool RokubiminiEthercatBusManager::addEthercatBus(soem_interface::EthercatBusBase* bus)
{
  if (bus == nullptr)
  {
    MELO_ERROR_STREAM("[RokubiminiEthercatBusManager::addEthercatBus] bus is nullptr")
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  const auto& it = buses_.find(bus->getName());
  if (it == buses_.end())
  {
    buses_.insert(std::make_pair(bus->getName(), std::unique_ptr<soem_interface::EthercatBusBase>(bus)));
    return true;
  }
  else
  {
    return false;
  }
}

bool RokubiminiEthercatBusManager::addEthercatBus(std::unique_ptr<soem_interface::EthercatBusBase> bus)
{
  if (bus == nullptr)
  {
    MELO_ERROR_STREAM("[RokubiminiEthercatBusManager::addEthercatBus] bus is nullptr")
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  const auto& it = buses_.find(bus->getName());
  if (it == buses_.end())
  {
    buses_.insert(std::make_pair(bus->getName(), std::move(bus)));
    return true;
  }
  else
  {
    return false;
  }
}

bool RokubiminiEthercatBusManager::startupAllBuses()
{
  bool success = startupCommunication();
  setBussesOperational();
  return success;
}

void RokubiminiEthercatBusManager::setBussesOperational()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_)
  {
    bus.second->setState(EC_STATE_OPERATIONAL);
  }
}

void RokubiminiEthercatBusManager::setBussesPreOperational()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_)
  {
    bus.second->setState(EC_STATE_PRE_OP);
  }
}

void RokubiminiEthercatBusManager::setBussesSafeOperational()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_)
  {
    bus.second->setState(EC_STATE_SAFE_OP);
  }
}

void RokubiminiEthercatBusManager::waitForState(const uint16_t state, const uint16_t slave, const std::string& busName,
                                                const unsigned int maxRetries, const double retrySleep)
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  if (busName.empty())
  {
    for (auto& bus : buses_)
    {
      bus.second->waitForState(state, slave, maxRetries, retrySleep);
    }
  }
  else
  {
    buses_.at(busName)->waitForState(state, slave, maxRetries, retrySleep);
  }
}

bool RokubiminiEthercatBusManager::startupCommunication()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_)
  {
    if (!bus.second->startup())
    {
      MELO_ERROR("Failed to startup bus %s.", bus.first.c_str());
      return false;
    }
  }
  return true;
}

void RokubiminiEthercatBusManager::readAllBuses()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_)
  {
    bus.second->updateRead();
  }
}

void RokubiminiEthercatBusManager::writeToAllBuses()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_)
  {
    bus.second->updateWrite();
  }
}

void RokubiminiEthercatBusManager::shutdownAllBuses()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_)
  {
    bus.second->shutdown();
  }
}

std::unique_ptr<EthercatBusBase> RokubiminiEthercatBusManager::extractBusByName(const std::string& name)
{
  std::unique_ptr<EthercatBusBase> bus_out = std::move(buses_.at(name));
  buses_.erase(name);
  return bus_out;
}

RokubiminiEthercatBusManager::BusMap RokubiminiEthercatBusManager::extractBuses()
{
  BusMap bus_map_out;

  for (auto& bus : buses_)
  {
    bus_map_out.insert(std::make_pair(bus.first, std::move(bus.second)));
  }

  buses_.clear();
  return bus_map_out;
}

}  // namespace ethercat
}  // namespace rokubimini