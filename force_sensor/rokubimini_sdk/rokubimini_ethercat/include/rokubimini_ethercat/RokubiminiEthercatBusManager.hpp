#pragma once

// std
#include <memory>
#include <mutex>

// rokubimini
#include <rokubimini_ethercat/soem_interface/EthercatBusBase.hpp>

#include <rokubimini/setup/Rokubimini.hpp>
#include <rokubimini/setup/Setup.hpp>

#include <rokubimini_ethercat/setup/RokubiminiEthercat.hpp>

#include <rokubimini_ethercat/RokubiminiEthercat.hpp>

#include <rokubimini_bus_manager/BusManager.hpp>
namespace rokubimini
{
namespace ethercat
{
using namespace soem_interface;

/**
 * @class RokubiminiEthercatBusManager
 *
 * @brief Inherits from RokubiminiBusManager. It's used for managing the Ethercat buses.
 *
 *
*/

class RokubiminiEthercatBusManager : public RokubiminiBusManager
{
public:
  using BusMap = std::unordered_map<std::string, std::unique_ptr<soem_interface::EthercatBusBase>>;

  RokubiminiEthercatBusManager() = default;
  ~RokubiminiEthercatBusManager() override = default;

  /**
*@brief      Adds an ethercat bus to the manager. The manager takes
*            ownership of the bus pointer
*
*@param      bus   Raw bus ptr
*
*@return     True if bus has been added successfully
*/
  bool addEthercatBus(soem_interface::EthercatBusBase* bus);

  /**
*@brief      Adds an ethercat bus to the manager. The manager takes
*            ownership of the bus pointer
*
*@param      bus   Unique bus ptr
*
*@return     True if bus has been added successfully
*/
  bool addEthercatBus(std::unique_ptr<soem_interface::EthercatBusBase> bus);

  /**
*@brief      Starts up all busses and puts them in operational mode
*
*@return     True if successful
*/
  bool startupAllBuses();

  /**
*@brief      Starts up all busses
*
*@return     True if successful
*/
  bool startupCommunication() override;

  /**
*@brief      Sets all busses to safe operational state
*/
  void setBussesSafeOperational();

  /**
*@brief      Sets all busses to pre operational state
*/
  void setBussesPreOperational();

  /**
*@brief      Sets all busses to operational state
*/
  void setBussesOperational();

  /**
*@brief      Waits for the slave to reach a state
*
*@param[in]  state       Ethercat state
*@param[in]  slave       Slave address, 0 = all slaves
*@param[in]  busName     The name of the bus
*@param[in]  maxRetries  The maximum retries
*@param[in]  retrySleep  The retry sleep
*/
  void waitForState(const uint16_t state, const uint16_t slave = 0, const std::string& busName = "",
                    const unsigned int maxRetries = 40, const double retrySleep = 0.001);

  /**
*@brief      Calls update read on all busses
*/
  void readAllBuses() override;

  /**
*@brief      Calls update write on all busses
*/
  void writeToAllBuses() override;

  /**
*@brief      Calls shutdown on all busses
*/
  void shutdownAllBuses() override;

  /**
*@brief      Returns a non owning bus pointer. The manager still manages all
*            the buses
*
*@param[in]  name  The bus name
*
*@return     A shared_ptr to the bus.
*/
  soem_interface::EthercatBusBase* getBusByName(const std::string& name) const
  {
    return buses_.at(name).get();
  }

  /**
*@brief      Returns an owning bus pointer. The manager is now not managing
*            this bus anymore
*
*@param[in]  name  The bus name
*
*@return     A unique_ptr to the bus.
*/
  std::unique_ptr<EthercatBusBase> extractBusByName(const std::string& name);

  /**
*@brief      Extracts all buses from the manager. The manager is now not
*            managing any buses anymore
*
*@return     Bus map
*/
  BusMap extractBuses();

  /**
*@fn virtual bool loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>> & rokubiminis)
*
*@brief Loads the Rokubimini Setups.
*
*This method is used for loading the Rokubimini Ethercat Setups
*assigned to the RokubiminiEthercatBusManager and then attaching
*them to the existing bus. It's given as input the vector with
*pointers to every rokubimini, from which the Ethercat
*implementation grubs the RokubiminiEthercat instances and attaches
*them to the bus. This is done by calling \a addRokubiminiToBus.
*
*
*@param rokubiminis The vector holding all the pointers to the existing Rokubiminis.
*
*@todo Find a better way to implement this:
*Right now we loop through every Rokubimini instance and check
*if it matches with the RokubiminiSetup instance which the
*BusManager has in its attachedRokubiminiSetups_. After the
*match has been done, the Rokubimini instance is passed to
*addRokubiminiToBus().
*/
  bool loadSetup(std::vector<std::shared_ptr<rokubimini::Rokubimini>>& rokubiminis) override;

  using RokubiminiBusManager::addRokubiminiToBus;

  /**
* @fn bool addRokubiminiToBus(
  RokubiminiEthercat *rokubimini,
  soem_interface::EthercatBusBase *bus,
  const std::shared_ptr<setup::RokubiminiEthercat> rokubiminiEthercatSetup) const
*
* @brief Adds a Rokubimini to Bus.
*
* This method is used for adding a Rokubimini Ethercat instance to
* the Ethercat Bus. This method also adds the EthercatSlave pointer
* to the Rokubimini Ethercat implementation.
*
* @param rokubimini The RokubiminiEthercat instance.
* @param bus The bus to which the RokubiminiEthercat will be attached.
* @param rokubiminiEthercatSetup The Rokubimini Ethercat Setup instance.
*/
  bool addRokubiminiToBus(RokubiminiEthercat* rokubimini, soem_interface::EthercatBusBase* bus,
                          const std::shared_ptr<setup::RokubiminiEthercat>& rokubiminiEthercatSetup) const;

  /**
*@fn virtual void doPreStartupActions()
*
*@brief Allows for Pre-Startup Actions from the EthercatBusManager.
*This method allows the EthercatBusManager to do some startup
*actions before the \a startup() method is called for
*every RokubiminiEthercat instance.
*
*/
  void setConfigMode() override;

  /**
*@fn virtual void doPostStartupActions()
*
*@brief Allows for Post-Startup Actions from the EthercatBusManager.
*
*This method allows the EthercatBusManager to do some startup
*actions after the \a startup() method is called for
*every RokubiminiEthercat instance.
*
*/

  void setRunMode() override;

protected:
  /**
*@var std::recursive_mutex busMutex_
*
*@brief Mutex prohibiting simultaneous access to EtherCAT bus
*manager.
*/
  std::recursive_mutex busMutex_;

  /**
*@var BusMap buses_
*
*@brief Map with all the Ethercat buses available.
*/
  BusMap buses_;
};

using RokubiminiEthercatBusManagerPtr = std::shared_ptr<RokubiminiEthercatBusManager>;

}  // namespace ethercat
}  // namespace rokubimini