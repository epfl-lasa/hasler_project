#pragma once

// std
#include <memory>
#include <mutex>

#include <rokubimini/setup/Setup.hpp>
#include <rokubimini/setup/Rokubimini.hpp>
#include <rokubimini/Rokubimini.hpp>

namespace rokubimini
{
/**
 * @class RokubiminiBusManager
 *
 * @brief The Rokubimini Bus Manager class.
 *
 * Base class which provides an extendable API. It's useful
 * for creating a bus manager with an existing
 * communication protocol.
 *
*/
class RokubiminiBusManager
{
public:
  /**
   * @fn RokubiminiBusManager()
   *
   * @brief Default constructor.
   *
   * This method constructs a \a RokubiminiBusManager and clears
   * the contents of the protected vector \a attachedRokubiminiSetups_.
   *
  */

  RokubiminiBusManager()
  {
    attachedRokubiminiSetups_.clear();
  };
  virtual ~RokubiminiBusManager() = default;

  /**
   * @fn virtual bool loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>> & rokubiminis)
   * @brief Loads the Rokubimini Setups.
   *
   * This method is used for loading the Rokubimini Setups assigned to the Bus Manager
   * and then attaching them to the existing bus. This method is implementation-specific,
   * therefore it's pure virtual in the \a RokubiminiBusManager class.
   * It's given as input the vector with pointers to every rokubimini, from which
   * the implementation should grub the ones which are relevenant to it and attach
   * them to the bus. This is done by calling \a addRokubiminiToBus.
   *
   * @param rokubiminis The vector holding all the pointers to the existing Rokubiminis.
  */

  virtual bool loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>>& rokubiminis) = 0;

  /**
   * @fn virtual bool addRokubiminiToBus(Rokubimini *rokubimini,
   * const std::shared_ptr<setup::Rokubimini> rokubiminiSetup) const
   *
   * @brief Adds a Rokubimini to Bus.
   *
   * This method is used for adding a Rokubimini (implementation-specific) to
   * the Bus (implementation-specific). This method is bound to each implementation
   * and the reason that it's here, is because of runtime polymorphism through
   * inheritance. It should be pointed that the methods in the children of this class,
   * add the implementation pointer to the Rokubimini implementation
   * (e.g. RokubiminiSerialImplPtr to the RokubiminiSerial instance).
   *
   * @param rokubimini The Rokubimini instance.
   * @param rokubiminiSetup The corresponding Rokubimini Setup instance.
  */

  virtual bool addRokubiminiToBus(Rokubimini* rokubimini,
                                  const std::shared_ptr<setup::Rokubimini>& rokubiminiSetup) const;

  /**
   * @fn virtual void addRokubiminiSetupToList(std::shared_ptr<setup::Rokubimini> rokubiminiSetup)
   *
   * @brief Adds a RokubiminiSetup to the internal list.
   *
   * This setter method is used for adding a Rokubimini Setup (implementation-specific) to
   * the internal list.
   *
   * @param rokubiminiSetup The RokubiminiSetup instance.
  */

  virtual void addRokubiminiSetupToList(const std::shared_ptr<setup::Rokubimini>& rokubiminiSetup);

  /**
   * @fn virtual bool startupCommunication()
   *
   * @brief Starts the communication through the bus.
   *
   * This method starts the communication with each attached Rokubimini
   * through the bus.
   *
  */

  virtual bool startupCommunication()
  {
    return true;
  };

  /**
   * @fn virtual void shutdownAllBuses()
   *
   * @brief Shuts down all the buses.
   *
   * This method shuts down all the buses
   * which have been created by the BusManager.
   *
  */

  virtual void shutdownAllBuses(){};

  /**
   * @fn virtual void writeToAllBuses()
   *
   * @brief Writes to all the buses.
   *
   * This method writes to all the buses
   * which have been created by the BusManager.
   *
  */

  virtual void writeToAllBuses(){};

  /**
   * @fn virtual void readAllBuses()
   *
   * @brief Reads all the buses.
   *
   * This method reads all the buses
   * which have been created by the BusManager.
  */

  virtual void readAllBuses(){};

  /**
   * @fn virtual void setConfigMode()
   *
   * @brief Sets the devices controlled from the BusManager to config mode.
   *
   * This method allows the BusManager to set the devices to configuration mode before the \a startup() method is called
   * for every Rokubimini instance.
   *
  */

  virtual void setConfigMode(){};

  /**
   * @fn virtual void setRunMode()
   *
   * @brief Sets the devices controlled from the BusManager to run mode.
   *
   * This method allows the BusManager to set the devices to run mode after the \a startup() method is called for every
   * Rokubimini instance.
   *
  */

  virtual void setRunMode(){};

  /**
   * @fn virtual void getRokubiminiSetups()
   *
   * @brief Gets all the Rokubimini Setup instances from the BusManager.
   *
   * This getter method returns all the Rokubimini Setups attached
   * to the BusManager.
   *
  */

  std::vector<std::shared_ptr<setup::Rokubimini>> getRokubiminiSetups()
  {
    return attachedRokubiminiSetups_;
  };

protected:
  /**
   * @var std::vector<std::shared_ptr<setup::Rokubimini>> attachedRokubiminiSetups_
   *
   * @brief List of attached Rokubimini Setups.
   *
   *
  */
  std::vector<std::shared_ptr<setup::Rokubimini>> attachedRokubiminiSetups_;
};

}  // namespace rokubimini