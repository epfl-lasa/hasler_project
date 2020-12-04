
#include <rokubimini_serial/RokubiminiSerialBusManager.hpp>

namespace rokubimini
{
namespace serial
{
bool RokubiminiSerialBusManager::loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>>& rokubiminis)
{
  // Loop through all Rokubiminis and create their buses.
  for (const auto& rokubimini_setup : attachedRokubiminiSetups_)
  {
    const auto rokubimini_serial_setup = std::dynamic_pointer_cast<setup::RokubiminiSerial>(rokubimini_setup);

    const std::string port = rokubimini_serial_setup->serialPort_;
    if (port.empty())
    {
      MELO_ERROR("[%s] The name of the port is empty.", rokubimini_setup->name_.c_str());
      return false;
    }

    // clear the existing list
    attachedRokubiminiSerials_.clear();
    for (const auto& rokubimini : rokubiminis)
    {
      if (rokubimini->getName() == rokubimini_serial_setup->name_)
      {
        // save each RokubiminiSerial for future operations.
        attachedRokubiminiSerials_.push_back(std::dynamic_pointer_cast<RokubiminiSerial>(rokubimini));
        auto rokubimini_serial = (RokubiminiSerial*)(rokubimini.get());
        if (!addRokubiminiToBus(rokubimini_serial, rokubimini_serial_setup))
        {
          return false;
        }
      }
    }
  }

  return true;
}

bool RokubiminiSerialBusManager::addRokubiminiToBus(
    RokubiminiSerial* rokubimini, const std::shared_ptr<setup::RokubiminiSerial>& rokubiminiSerialSetup) const
{
  auto impl_ptr = std::make_shared<RokubiminiSerialImpl>(
      rokubiminiSerialSetup->name_, rokubiminiSerialSetup->serialPort_, rokubiminiSerialSetup->baudRate_);

  rokubimini->setImplPointer(impl_ptr);
  return true;
}

bool RokubiminiSerialBusManager::startupCommunication()
{
  for (const auto& rokubimini_serial : attachedRokubiminiSerials_)
  {
    if (!rokubimini_serial->init())
    {
      MELO_ERROR("[%s] The Serial device could not initialize", rokubimini_serial->getName().c_str());
      return false;
    }
  }
  return true;
}

void RokubiminiSerialBusManager::setConfigMode()
{
  for (const auto& rokubimini_serial : attachedRokubiminiSerials_)
  {
    if (!rokubimini_serial->setConfigMode())
    {
      MELO_ERROR("[%s] The Serial device could not switch to configuration mode", rokubimini_serial->getName().c_str());
    }
  }
}
void RokubiminiSerialBusManager::setRunMode()
{
  for (const auto& rokubimini_serial : attachedRokubiminiSerials_)
  {
    if (!rokubimini_serial->setRunMode())
    {
      MELO_ERROR("[%s] The Serial device could not switch to run mode", rokubimini_serial->getName().c_str());
    }
  }
}
}  // namespace serial
}  // namespace rokubimini