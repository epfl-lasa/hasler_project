// signal handler
#include <signal_handler/SignalHandler.hpp>
#include <rokubimini_manager/Manager.hpp>
#include <rokubimini_factory/RokubiminiFactory.hpp>
namespace rokubimini
{
RokubiminiManager::RokubiminiManager(const bool standalone, const bool installSignalHandler, const double timeStep)
  : rokubiminis_(), busManagers_(), standalone_(standalone), timeStep_(timeStep)
{
  if (installSignalHandler)
  {
    signal_handler::SignalHandler::bindAll(&RokubiminiManager::handleSignal, this);
  }
}
Rokubimini* RokubiminiManager::getRokubimini(const std::string& name) const
{
  for (auto& rokubimini : rokubiminis_)
  {
    if (rokubimini->getName() == name)
    {
      return static_cast<Rokubimini*>(rokubimini.get());
    }
  }
  MELO_ERROR_STREAM("Could not find Rokubimini with the name: " << name);
  return nullptr;
}
std::vector<Rokubimini*> RokubiminiManager::getRokubiminis() const
{
  std::vector<Rokubimini*> rokubimini_out;
  rokubimini_out.reserve(rokubiminis_.size());
  for (const auto& rokubimini : rokubiminis_)
  {
    rokubimini_out.emplace_back(rokubimini.get());
  }
  return rokubimini_out;
}
bool RokubiminiManager::loadSetup(const std::string& configurationFile)
{
  auto setup = rokubimini::loadBusSetup(configurationFile, busManagers_);
  if (setup == nullptr)
  {
    MELO_ERROR_STREAM("Could not load Bus Setup from file: " << configurationFile);
    return false;
  }
  return createRokubiminisFromSetup(setup);
}
bool RokubiminiManager::createRokubiminisFromSetup(const rokubimini::setup::SetupPtr& setup)
{
  rokubiminis_.clear();
  for (const auto& rokubimini_setup : setup->rokubiminis_)
  {
    if (!createAndConfigureRokubimini(rokubimini_setup))
    {
      return false;
    }
  }
  return loadBusManagersSetup();
}

bool RokubiminiManager::loadBusManagersSetup()
{
  for (const auto& bus_manager_ptr : busManagers_)
  {
    if (!bus_manager_ptr->loadSetup(rokubiminis_))
      return false;
  }
  return true;
}
bool RokubiminiManager::RokubiminiManager::createAndConfigureRokubimini(
    const std::shared_ptr<rokubimini::setup::Rokubimini>& rokubiminiSetup)
{
  auto rokubimini = rokubimini::createRokubiminiFactory(rokubiminiSetup);
  if (!rokubimini->loadRokubiminiSetup(rokubiminiSetup.get()))
  {
    return false;
  }
  if (!addRokubimini(rokubimini))
  {
    return false;
  }
  return true;
}

bool RokubiminiManager::addRokubimini(Rokubimini* rokubimini)
{
  const std::string name = rokubimini->getName();
  if (rokubiminiExists(name))
  {
    MELO_ERROR_STREAM("Cannot add Rokubimini with name '" << name << "', because it already exists.");
    return false;
  }
  rokubiminis_.push_back(std::shared_ptr<Rokubimini>(rokubimini));
  return true;
}
bool RokubiminiManager::rokubiminiExists(const std::string& name) const
{
  for (const auto& rokubimini : rokubiminis_)
  {
    if (rokubimini->getName() == name)
    {
      return true;
    }
  }
  return false;
}
bool RokubiminiManager::startup()
{
  if (isRunning_)
  {
    MELO_WARN_STREAM("Cannot start up, Rokubimini Manager is already running.");
    return false;
  }
  MELO_DEBUG_STREAM("Starting up Rokubimini Manager ...");
  if (busManagers_.empty())
  {
    throw message_logger::log::melo_fatal("Cannot start up, a communication manager has not been set.");
  }
  for (const auto& rokubimini : rokubiminis_)
  {
    rokubimini->startupWithoutCommunication();
  }
  for (const auto& bus_manager_ptr : busManagers_)
  {
    if (!bus_manager_ptr->startupCommunication())
    {
      // auto rokubiminis = mapOfRokubiminisToBusManagers_.find(move(busManagerPtr))->second;
      // search for the rokubiminis of the failed busManager
      for (const auto& rokubimini_setup : bus_manager_ptr->getRokubiminiSetups())
      {
        auto rokubimini = getRokubimini(rokubimini_setup->name_);
        rokubimini->shutdownWithoutCommunication();
      }
      return false;
    }
  }
  for (const auto& bus_manager_ptr : busManagers_)
  {
    bus_manager_ptr->setConfigMode();
  }
  for (const auto& rokubimini : rokubiminis_)
  {
    rokubimini->startupWithCommunication();
  }
  for (const auto& bus_manager_ptr : busManagers_)
  {
    bus_manager_ptr->setRunMode();
  }
  if (standalone_)
  {
    any_worker::WorkerOptions update_worker_options;
    update_worker_options.callback_ = std::bind(&RokubiminiManager::updateWorkerCb, this, std::placeholders::_1);
    update_worker_options.defaultPriority_ = 90;
    update_worker_options.name_ = "RokubiminiManager::updateWorker";
    update_worker_options.timeStep_ = timeStep_;
    update_worker_options.enforceRate_ = true;
    updateWorker_.reset(new any_worker::Worker(update_worker_options));
    if (!updateWorker_->start())
    {
      throw message_logger::log::melo_fatal("Update worker could not be started.");
    }
  }
  isRunning_ = true;
  return true;
}
bool RokubiminiManager::update()
{
  updateCommunicationManagerReadMessages();
  updateProcessReadings();
  updateSendStagedCommands();
  updateCommunicationManagerWriteMessages();
  return true;
}
void RokubiminiManager::updateCommunicationManagerReadMessages()
{
  for (const auto& bus_manager_ptr : busManagers_)
  {
    bus_manager_ptr->readAllBuses();
  }
}
void RokubiminiManager::updateProcessReadings()
{
  for (const auto& rokubimini : rokubiminis_)
  {
    rokubimini->updateProcessReading();
  }
}
void RokubiminiManager::updateSendStagedCommands()
{
  for (const auto& rokubimini : rokubiminis_)
  {
    rokubimini->updateSendStagedCommand();
  }
}
void RokubiminiManager::updateCommunicationManagerWriteMessages()
{
  for (const auto& bus_manager_ptr : busManagers_)
  {
    bus_manager_ptr->writeToAllBuses();
  }
}
void RokubiminiManager::shutdown()
{
  if (!isRunning_)
  {
    MELO_WARN_STREAM("Cannot shut down, Rokubimini Manager is not running.");
    return;
  }
  // If not in standalone mode, the exterior worker is already shutdown at this point.
  // For this reason, a temporary worker has to take over the job.
  if (!standalone_)
  {
    any_worker::WorkerOptions update_worker_options;
    update_worker_options.callback_ = std::bind(&RokubiminiManager::updateWorkerCb, this, std::placeholders::_1);
    update_worker_options.defaultPriority_ = 90;
    update_worker_options.name_ = "RokubiminiManager::updateWorker (shutdown)";
    update_worker_options.timeStep_ = timeStep_;
    update_worker_options.enforceRate_ = true;
    updateWorker_.reset(new any_worker::Worker(update_worker_options));
    if (!updateWorker_->start())
    {
      MELO_ERROR_STREAM("Update worker (shutdown) could not be started.");
    }
  }
  for (const auto& rokubimini : rokubiminis_)
  {
    rokubimini->shutdownWithCommunication();
  }
  // Stop the worker here (in both modes, standalone and not).
  updateWorker_->stop(true);
  for (const auto& bus_manager_ptr : busManagers_)
  {
    bus_manager_ptr->shutdownAllBuses();
  }
  for (const auto& rokubimini : rokubiminis_)
  {
    rokubimini->shutdownWithoutCommunication();
  }
  isRunning_ = false;
}

bool RokubiminiManager::updateWorkerCb(const any_worker::WorkerEvent& event)
{
  return update();
}
void RokubiminiManager::RokubiminiManager::requestShutdown()
{
  shutdownRequested_ = true;
  // shutdown processes which take a longer period of time
}
void RokubiminiManager::handleSignal(const int signum)
{
  MELO_INFO_STREAM("Received signal (" << signum << "), requesting shutdown ...");
  requestShutdown();
  if (signum == SIGSEGV)
  {
    signal(signum, SIG_DFL);
    kill(getpid(), signum);
  }
}
}  // namespace rokubimini