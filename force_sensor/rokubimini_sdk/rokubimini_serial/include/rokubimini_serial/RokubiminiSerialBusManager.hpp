#pragma once

// std
#include <memory>
#include <mutex>

// rokubimini
#include <rokubimini/setup/Rokubimini.hpp>
#include <rokubimini/setup/Setup.hpp>
#include <rokubimini_serial/setup/RokubiminiSerial.hpp>
#include <rokubimini_serial/RokubiminiSerial.hpp>

#include <rokubimini_bus_manager/BusManager.hpp>
namespace rokubimini
{
namespace serial
{
/**
 * @class RokubiminiSerialBusManager
 *
 * @brief Inherits from RokubiminiBusManager. It's used for managing the serial bus.
 *
 * Although there isn't such a thing called "serial bus", it's used
 * for compliance with the inheritance scheme. The bus-specifics
 * don't exist here and the only things implemented are the way to
 * have a list of RokubiminiSerial Setups and the way for creating
 * RokubiminiSerialImpl objects.
 *
*/
class RokubiminiSerialBusManager : public RokubiminiBusManager
{
public:
  /**
   * @fn RokubiminiSerialBusManager()
   *
   * @brief Default constructor of RokubiminiSerialBusManager.
   *
  */
  RokubiminiSerialBusManager() = default;
  ~RokubiminiSerialBusManager() override = default;

  /**
   * @fn bool loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>> &rokubiminis)
   *
   * @brief Checks for non-empty serial port names and calls the \a
   * addRokubiminiToBus() method.
   *
   * @param rokubiminis The Rokubimini instances. Only the Serial
   * ones will be chosen for creating RokubiminiSerialImpl objects.
   * @return True if the checks and the creation of the
   * RokubiminiSerialImpl objects are successful.
   *
   * @todo Find a better way to implement this:
   * Right now we loop through every Rokubimini instance and check
   * if it matches with the RokubiminiSetup instance which the
   * BusManager has in its attachedRokubiminiSetups_. After the
   * match has been done, the Rokubimini instance is passed to
   * addRokubiminiToBus().
  */
  bool loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>>& rokubiminis) override;

  /**
   * @fn bool addRokubiminiToBus(
      RokubiminiSerial *rokubimini,
      const std::shared_ptr<setup::RokubiminiSerial> rokubiminiSerialSetup) const
   *
   * @brief Creates a RokubiminiSerial Implementation object for
   * the @param rokubimini given.
   *
   * @param rokubimini The Rokubimini Serial instance for which an
   * implementation will be created.
   * @param rokubiminiSerialSetup The Rokubimini Serial Setup
   * instance from which the serial-specifics (port, baud rate)
   * will be drawn.
   *
   * @return True if the creation of the
   * RokubiminiSerialImpl object is successful.
   *
  */
  using RokubiminiBusManager::addRokubiminiToBus;
  bool addRokubiminiToBus(RokubiminiSerial* rokubimini,
                          const std::shared_ptr<setup::RokubiminiSerial>& rokubiminiSerialSetup) const;

  /**
   * @fn void setConfigMode()
   *
   * @brief Sets all the serial devices in config mode.
   *
  */
  void setConfigMode() override;

  /**
   * @fn void setRunMode()
   *
   * @brief Sets all the serial devices in run mode.
   *
  */
  void setRunMode() override;

  /**
   * @fn bool startupCommunication()
   *
   * @brief Initializes the communication with the serial devices.
   *
   * @return True if the operation was successful.
  */
  bool startupCommunication() override;

  std::vector<std::shared_ptr<RokubiminiSerial>> attachedRokubiminiSerials_;
};

using RokubiminiSerialBusManagerPtr = std::shared_ptr<RokubiminiSerialBusManager>;

}  // namespace serial
}  // namespace rokubimini