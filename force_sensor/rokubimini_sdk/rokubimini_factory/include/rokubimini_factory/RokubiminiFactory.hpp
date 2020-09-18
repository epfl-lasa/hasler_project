#include <rokubimini/Rokubimini.hpp>
#include <rokubimini_bus_manager/BusManager.hpp>

namespace rokubimini
{
typedef enum BusType { SERIAL = 1, ETHERCAT = 2 } busType;

/**
 * @fn rokubimini::setup::SetupPtr loadBusSetup(const std::string &configurationFile,
 * std::vector<std::unique_ptr<RokubiminiBusManager>> &busManagers)
 *
 * @brief Creates a \a Setup from the configuration file configurationFile.
 *
 * This function creates a \a Setup, loads into it the bus
 * setups from the file given, creates the relevant list with the \a
 * busManagers and returns a pointer to the \a Setup created.
 *
 * @param configurationFile The file to load the configuration from.
 * @param busManagers The list of \a busManagers which will be filled
 * based on the contents of the configuration file.
 *
*/
rokubimini::setup::SetupPtr loadBusSetup(const std::string& configurationFile,
                                         std::vector<std::unique_ptr<RokubiminiBusManager>>& busManagers);

/**
 * @fn busType parseProductCodeFromYamlNode(const yaml_tools::YamlNode &yamlNode)
 *
 * @brief Parses the Product Code from the yamlNode.
 *
 * @param yamlNode The Yaml Node to be parsed.
*/

busType parseProductCodeFromYamlNode(const yaml_tools::YamlNode& yamlNode);

/**
 * @fn rokubimini::Rokubimini *createRokubiminiFactory(const std::shared_ptr<rokubimini::setup::Rokubimini>
 * rokubiminiSetup)
 *
 * @brief Creates an implementation-specific Rokubimini instance
 *
 * This function creates a Rokubimini instance based on the
 * rokubiminiSetup. The product code is checked and based on this
 * the relevant Rokubimini instance is created.
 *
 * @param rokubiminiSetup The implementation-specific Rokubimini Setup.
 *
*/

rokubimini::Rokubimini* createRokubiminiFactory(const std::shared_ptr<rokubimini::setup::Rokubimini>& rokubiminiSetup);

/**
 * @fn rokubimini::setup::SetupPtr createSetup()
 *
 * @brief Creates an empty generic Setup. This
 * will have in a list all the Rokubimini Setups.
 *
 *
*/

rokubimini::setup::SetupPtr createSetup();

/**
 * @fn bool checkAndLoadBusSetupFromFile(rokubimini::setup::SetupPtr setup, const std::string &setupFile,
 * std::vector<std::unique_ptr<RokubiminiBusManager>> &busManagers)
 *
 * @brief Checks and loads the Bus Setup from the setupFile file.
 *
 * This function checks the file given and if everything is ok, it calls
 * \a loadSetupFromFile().
 *
 * @param setup The Setup to be filled with all the Rokubimini Setups.
 * @param setupFile The file to load the Bus Setup from.
 * @param busManagers The list with the Bus Managers to be filled.
 *
*/

bool checkAndLoadBusSetupFromFile(const rokubimini::setup::SetupPtr& setup, const std::string& setupFile,
                                  std::vector<std::unique_ptr<RokubiminiBusManager>>& busManagers);

/**
 * @fn void loadBusSetupFromFile(rokubimini::setup::SetupPtr setup, const std::string &setupFile,
 * std::vector<std::unique_ptr<RokubiminiBusManager>> &busManagers)
 *
 * @brief Parses the Yaml file setupFile  and creates all
 * the Rokubimini Setups along with the corresponding Bus Managers.
 * Calls \a createRokubiminiSetupFromFile().
 *
 *
 * @param setup The Setup to be filled with all the Rokubimini Setups.
 * @param setupFile The file to load the Bus Setup from.
 * @param busManagers The list with the Bus Managers to be filled.
 *
*/

void loadBusSetupFromFile(const rokubimini::setup::SetupPtr& setup, const std::string& setupFile,
                          std::vector<std::unique_ptr<RokubiminiBusManager>>& busManagers);

/**
 * @fn std::shared_ptr<rokubimini::setup::Rokubimini> createRokubiminiSetupFromFile(const yaml_tools::YamlNode
 * &yamlNode, const std::string &setupFile, std::vector<std::unique_ptr<RokubiminiBusManager>> &busManagers)
 *
 * @brief Creates a Rokubimini Setup and Bus Managers.
 *
 * This function actually creates the implementation-specific
 * Rokubimini Setup based on the Product Code given in the setup file
 * for each Rokubimini. Then it creates a Bus Manager (if it doesn't
 * exist) and adds the newly created Rokubimini Setup to the list of
 * the Bus Manager. Finally it adds the Bus Manager to the list which
 * the Manager keeps.
 *
 * @param yamlNode The Yaml Node to be parsed.
 * @param setupFile The file to load the Bus Setup from.
 * @param busManagers The list with the Bus Managers to be filled.
 *
 * @todo Find a better way to implement this:
 * We need to check if a unique_ptr to
 * RokubiminiEthercatBusManager and RokubiminiSerialBusManager already
 * exists. Otherwise
 * make_unique will create a new unique_ptr to a
 * RokubiminiEthercatBusManager and RokubiminiSerialBusManager (by transferring
 * ownership from a previously created unique_ptr).
 * This probably will lead to memory leaks. I checked
 * it without the following process (only with
 * make_unique) with 4 Rokubimis (2 Serial and 2
 * Ethercat) and there were 4 BusManagers created (which
 * is wrong).
*/

std::shared_ptr<rokubimini::setup::Rokubimini>
createRokubiminiSetupFromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile,
                              std::vector<std::unique_ptr<RokubiminiBusManager>>& busManagers);
}  // namespace rokubimini