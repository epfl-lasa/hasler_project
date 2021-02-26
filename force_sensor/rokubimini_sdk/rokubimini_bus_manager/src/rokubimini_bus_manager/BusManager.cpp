
// #include <rokubimini/setup/Setup.hpp>
// #include <rokubimini/setup/Rokubimini.hpp>
#include <rokubimini_bus_manager/BusManager.hpp>

namespace rokubimini
{
bool RokubiminiBusManager::addRokubiminiToBus(Rokubimini* rokubimini,
                                              const std::shared_ptr<setup::Rokubimini>& rokubiminiSetup) const
{
  return true;
};
void RokubiminiBusManager::addRokubiminiSetupToList(const std::shared_ptr<setup::Rokubimini>& rokubiminiSetup)
{
  attachedRokubiminiSetups_.push_back(rokubiminiSetup);
};

}  // namespace rokubimini