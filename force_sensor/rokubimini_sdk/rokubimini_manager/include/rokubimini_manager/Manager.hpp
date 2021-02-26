#pragma once

#include <vector>
#include <mutex>
#include <unordered_map>
#include <any_worker/Worker.hpp>

#include <rokubimini/setup/Setup.hpp>
#include <rokubimini/setup/Rokubimini.hpp>

#include <rokubimini/Rokubimini.hpp>
#include <rokubimini_bus_manager/BusManager.hpp>

namespace rokubimini
{
/**
 * @class BaseManager
 *
 * @brief The Base Manager class.
 *
 * Base abstract class. It provides the basic API
 * needed for a Manager which loads a configuration file
 * and starts/updates/shuts down the communication with
 * the devices.
 *
*/

class BaseManager
{
public:
  /**
   * @fn BaseManager()
   *
   * @brief Default constructor.
   *
   * This method constructs a \a BaseManager.
   *
  */
  BaseManager(){};

  virtual ~BaseManager() = default;

  /**
   * @fn virtual bool loadSetup(const std::string &setupFile)
   * @brief Loads the Setup from the setup file given.
   *
   * This pure virtual method is used as a reference for future
   * implementations.
   *
   * @param setupFile The file to load the Setup from.
  */

  virtual bool loadSetup(const std::string& setupFile = "") = 0;

  /**
   * @fn virtual bool startup()
   *
   * @brief Starts up the communication  with the devices.
   *
   * This pure virtual method is used as a reference for future
   * implementations.
   *
  */

  virtual bool startup() = 0;

  /**
   * @fn virtual bool update()
   *
   * @brief Updates (Readings and Commands) the communication  with the devices.
   *
   * This pure virtual method is used as a reference for future
   * implementations.
   *
  */

  virtual bool update() = 0;

  /**
   * @fn virtual bool shutdown()
   *
   * @brief Shuts down the communication  with the devices.
   *
   * This pure virtual method is used as a reference for future
   * implementations.
   *
  */
  virtual void shutdown() = 0;
};

/**
 * @class RokubiminiManager
 *
 * @brief The Rokubimini Manager class.
 *
 * A class that provides a general implementation
 * of a Manager for the Rokubimini devices. It
 * implements the base functionalities from its
 * parent (load, startup, update, shutdown) and
 * provides a generic and medium-ignorant way of
 * doing so.
 *
*/

class RokubiminiManager : public BaseManager
{
public:
  RokubiminiManager() = delete;

  /**
   * @fn RokubiminiManager(
      const bool standalone,
      const bool installSignalHandler,
      const double timeStep)
   *
   * @brief Custom constructor. The default constructor is deleted.
   *
   * This method constructs a \a RokubiminiManager and starts up
   * a Signal Handler upon request.
   *
   * @param standalone Boolean that defines whether the Manager
   * should run as a standalone program.
   * @param installSignalHandler Boolean that defines whether there
   * will be a signal handler installed.
   * @param timeStep Boolean that defines the time step between two
   * consecutive updates(?).
  */

  RokubiminiManager(const bool standalone, const bool installSignalHandler, const double timeStep = 0.01);

  ~RokubiminiManager() override = default;

  /**
   * @fn Rokubimini *getRokubimini(const std::string &name) const
   * @brief Returns the Rokubimini instance with name \a name.
   *
   * This method is used for getting a Rokubimini instance based on
   * its name. If there isn't such a Rokubimini in the Manager's list,
   * a \a nullptr is returned.
   *
   * @param name The name of the Rokubimini to be found.
  */

  Rokubimini* getRokubimini(const std::string& name) const;

  /**
   * @fn std::vector<Rokubimini *> getRokubiminis() const
   *
   * @brief Returns all the Rokubimini instances located in the
   * Manager's list.
   *
   * This getter method is used for getting all the Rokubimini
   * instances of the Manager.
   *
  */

  std::vector<Rokubimini*> getRokubiminis() const;

  /**
   * @fn bool loadSetup(const std::string &setupFile)
   *
   * @brief Loads the Setup located in the \a setupFile.
   *
   * This method is used for loading the Setup from a file
   * given as parameter. It starts by parsing the \a setupFile
   * and acquiring general information for each to-be-set
   * Rokubimini (e.g. \a name, \a configuration_file), as well
   * as specific information (e.g. \a ethercat_bus or \a port).
   * This process is done through calling functions from the
   * \a rokubimini_factory which knows the implementation-specific
   * stuff. Then, it creates Rokubiminis based on the
   * implementation-specific information located in the \a setupFile
   * (by calling again \rokubimini_factory functions) and loads
   * the configuration found in the \a configuration_file which is
   * given for each. Finally it calls the relevant
   * \a BusManager to attach it to the bus. This process
   * is also done through the \a rokubimini_factory.
   *
   * @param setupFile The file to load the setup from.
  */

  bool loadSetup(const std::string& setupFile = "") override;

  /**
   * @fn bool startup()
   *
   * @brief Starts the communication with all the Rokubimini devices.
   *
   * This method starts the communication with all the Rokubimini
   * devices. Also some funcionalities of the \a BusManagers are
   * called between the startup phases of the Rokubiminis.
   *
  */

  bool startup() override;

  /**
   * @fn bool update()
   *
   * @brief Updates with new values from/to the Rokubiminis.
   *
   * This method updates the Manager with new values
   * (\a Readings) from the Rokubimini devices and
   * updates the Rokubimini devices with new \a Commands
   * if there are any. Also there are calls to
   * optional implementation of \a readAllBuses and
   * \a writeToAllBuses of each \a BusManager implementation.
   *
  */

  bool update() override;

  /**
   * @fn void shutdown()
   *
   * @brief Shuts down everything.
   *
   * Shuts down all the Rokubimini devices
   * and the buses from their relative BusManagers.
   * Also stops the \a updateWorker thread that is
   * running in parallel.
   *
  */

  void shutdown() override;

  /**
   * @fn void updateProcessReadings()
   *
   * @brief Gets the Readings from each Rokubimini device.
   *
   * This method gets the Readings from each Rokubimini device.
   * It calls the \a updateProcessReading method of each Rokubimini
   * instance.
  */

  void updateProcessReadings();

  /**
   * @fn void updateSendStagedCommands()
   *
   * @brief Sends Staged Commands to each Rokubimini device.
   *
   * This method gets the Readings from each Rokubimini device.
   * It calls the \a updateSendStagedCommand method of each Rokubimini
   * instance.
   *
  */

  void updateSendStagedCommands();

  /**
   * @fn void updateCommunicationManagerWriteMessages()
   *
   * @brief Instructs each \a BusManager to write the messages.
   *
   * This method allows each BusManager to write the messages
   * created to its attached buses.
   *
  */

  void updateCommunicationManagerWriteMessages();

  /**
   * @fn void updateCommunicationManagerReadMessages
   *
   * @brief Instructs each \a BusManager to read the messages.
   *
   * This method allows each BusManager to read the messages
   * created to its attached buses.
  */

  void updateCommunicationManagerReadMessages();

protected:
  /**
   * @fn bool addRokubimini(Rokubimini *rokubimini)
   *
   * @brief Adds a Rokubimini instance to the list.
   *
   * @param rokubimini The Rokubimini instance.
  */

  bool addRokubimini(Rokubimini* rokubimini);

  /**
   * @fn bool rokubiminiExists(const std::string &name) const
   *
   * @brief Decides if a specific Rokubimini instance exists in the
   * list.
   *
   * @param name Name to search in the list.
  */

  bool rokubiminiExists(const std::string& name) const;

  /**
   * @fn bool updateWorkerCb(const any_worker::WorkerEvent &event)
   *
   * @brief Callback method which when triggered calls the
   * \a update() method.
   *
   * @param event Event triggering the callback.
   *
   *
  */

  bool updateWorkerCb(const any_worker::WorkerEvent& event);

  /**
   * @fn void requestShutdown()
   *
   * @brief Sets a flag that represents that shutdown has been
   * requested.
   *
   *
  */

  void requestShutdown();

  /**
   * @fn bool createRokubiminisFromSetup(const rokubimini::setup::SetupPtr &setup)
   *
   * @brief Creates the Rokubimini instances from a Setup.
   *
   * @param setup The Setup needed for creating the Rokubimini
   * instances.
  */

  bool createRokubiminisFromSetup(const rokubimini::setup::SetupPtr& setup);

  /**
   * @fn bool loadBusManagersSetup()
   *
   * @brief Calls the \a loadSetup() of each Bus Manager.
   *
  */

  bool loadBusManagersSetup();

  /**
   * @fn bool createAndConfigureRokubimini(const std::shared_ptr<rokubimini::setup::Rokubimini> rokubiminiSetup)
   *
   * @brief Creates an implementation-specific Rokubimini instance
   * (through the \a rokubimini_factory), loads the
   * \a RokubiminiSetup to it and adds the new instance to the list.
   *
   * @param rokubiminiSetup The \a RokubiminiSetup which will be loaded to a new Rokubimini instance.
   *
  */

  bool createAndConfigureRokubimini(const std::shared_ptr<rokubimini::setup::Rokubimini>& rokubiminiSetup);

  /**
   * @var std::shared_ptr<any_worker::Worker> updateWorker_
   *
   * @brief A \a Worker instance that handles the update process.
   *
   *
  */

  std::shared_ptr<any_worker::Worker> updateWorker_{ nullptr };

  /**
   * @var std::atomic<bool> isRunning_
   *
   * @brief Boolean that specifies if the Manager is already running.
   *
   *
  */

  std::atomic<bool> isRunning_{ false };

  /**
   * @var std::atomic<bool> shutdownRequested_
   *
   * @brief Boolean that specifies if shutdown has been requested.
   *
   *
  */

  std::atomic<bool> shutdownRequested_{ false };

  /**
   * @var std::vector<std::shared_ptr<Rokubimini>> rokubiminis_
   *
   * @brief List of existing \a Rokubimini instances.
   *
   *
  */

  std::vector<std::shared_ptr<Rokubimini>> rokubiminis_;
  // std::unordered_map<std::unique_ptr<RokubiminiBusManager>, std::vector<std::unique_ptr<Rokubimini>>>
  // mapOfRokubiminisToBusManagers_;

  /**
   * @var std::vector<std::unique_ptr<RokubiminiBusManager>> busManagers_
   *
   * @brief List of existing \a busManagers.
   *
   *
  */
  std::vector<std::unique_ptr<RokubiminiBusManager>> busManagers_;

  /**
   * @fn void handleSignal(const int signum)
   *
   * @brief Handles the signal received. Specifically
   * requests shuts down if the \a signum is SIGSEGV.
   *
   * @param signum The number of the signal.
   *
  */
  void handleSignal(const int signum);

private:
  bool standalone_{ true };
  double timeStep_{ 0.0 };
  std::string configurationFile_{ "" };
};
}  // namespace rokubimini
