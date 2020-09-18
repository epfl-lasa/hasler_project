

#include <rokubimini/setup/Setup.hpp>

#include <rokubimini_factory/RokubiminiFactory.hpp>
#include <rokubimini_serial/RokubiminiSerial.hpp>
#include <rokubimini_serial/RokubiminiSerialBusManager.hpp>
#include <rokubimini_ethercat/RokubiminiEthercat.hpp>
#include <rokubimini_ethercat/RokubiminiEthercatBusManager.hpp>
#include <utility>

using namespace std;
using namespace rokubimini::ethercat;
using namespace rokubimini::serial;
namespace rokubimini
{
std::shared_ptr<rokubimini::setup::Rokubimini>
createRokubiminiSetupFromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile,
                              vector<unique_ptr<RokubiminiBusManager>>& busManagers)
{
  std::shared_ptr<rokubimini::setup::Rokubimini> rokubimini_setup;
  busType product_code;
  if (yamlNode.hasKey("product_code"))
  {
    product_code = parseProductCodeFromYamlNode(yamlNode);
    switch (product_code)
    {
      case ETHERCAT:
      {
        auto ethercat_setup = make_shared<rokubimini::ethercat::setup::RokubiminiEthercat>();
        ethercat_setup->fromFile(yamlNode, setupFile);
        // check TODO in doxygen.
        bool bus_manager_found = false;
        for (auto& bus_manager : busManagers)
        {
          // To check if the busManager is a
          // RokubiminiEthercatBusManager, we use it's first
          // attached RokubiminiSetup's productCode.
          if (bus_manager->getRokubiminiSetups()[0]->productCode_ == ETHERCAT)
          {
            bus_manager->addRokubiminiSetupToList(ethercat_setup);
            bus_manager_found = true;
            break;
          };
        }
        if (!bus_manager_found)
        {
          auto bus_manager = make_unique<RokubiminiEthercatBusManager>();
          bus_manager->addRokubiminiSetupToList(ethercat_setup);
          busManagers.push_back(move(bus_manager));
        }
        rokubimini_setup = static_pointer_cast<rokubimini::setup::Rokubimini>(ethercat_setup);
        break;
      }
      case SERIAL:
      {
        auto serial_setup = make_shared<rokubimini::serial::setup::RokubiminiSerial>();
        serial_setup->fromFile(yamlNode, setupFile);

        // check TODO in doxygen.
        bool bus_manager_found = false;
        for (auto& bus_manager : busManagers)
        {
          // To check if the busManager is a
          // RokubiminiSerialBusManager, we use it's first
          // attached RokubiminiSetup's productCode.
          if (bus_manager->getRokubiminiSetups()[0]->productCode_ == SERIAL)
          {
            bus_manager->addRokubiminiSetupToList(serial_setup);
            bus_manager_found = true;
            break;
          };
        }
        if (!bus_manager_found)
        {
          auto bus_manager = make_unique<RokubiminiSerialBusManager>();
          bus_manager->addRokubiminiSetupToList(serial_setup);
          busManagers.push_back(move(bus_manager));
        }
        rokubimini_setup = static_pointer_cast<rokubimini::setup::Rokubimini>(serial_setup);
        break;
      }
      default:
        MELO_ERROR("Wrongly typed product code in a rokubimini setup!");
        rokubimini_setup = nullptr;
        break;
    }
  }
  else
  {
    MELO_ERROR("Could not find product code of one Rokubimini in the Bus Setup file!");
    rokubimini_setup = nullptr;
  }
  return rokubimini_setup;
}

busType parseProductCodeFromYamlNode(const yaml_tools::YamlNode& yamlNode)
{
  auto product_code = yamlNode["product_code"].as<std::uint32_t>();
  return (busType)product_code;
}

rokubimini::Rokubimini* createRokubiminiFactory(const std::shared_ptr<rokubimini::setup::Rokubimini>& rokubiminiSetup)
{
  rokubimini::Rokubimini* rokubimini = nullptr;
  auto product_code = rokubiminiSetup->productCode_;
  switch (product_code)
  {
    case ETHERCAT:
    {
      rokubimini = new RokubiminiEthercat();
      break;
    }
    case SERIAL:
    {
      rokubimini = new RokubiminiSerial();
      break;
    }
    default:
      MELO_ERROR("Wrongly typed product code in a rokubimini setup!");
      break;
  }
  return (rokubimini::Rokubimini*)rokubimini;
}
void loadBusSetupFromFile(const rokubimini::setup::SetupPtr& setup, const std::string& setupFile,
                          vector<unique_ptr<RokubiminiBusManager>>& busManagers)
{
  yaml_tools::YamlNode yaml_node = yaml_tools::YamlNode::fromFile(setupFile);
  if (yaml_node.hasKey("rokubiminis"))
  {
    // Clear the rokubiminis first.
    setup->rokubiminis_.clear();

    const yaml_tools::YamlNode rokubiminis = yaml_node["rokubiminis"];

    for (const auto& i : rokubiminis)
    {
      auto rokubimini = createRokubiminiSetupFromFile(i, setupFile, busManagers);
      setup->rokubiminis_.push_back(rokubimini);
    }
    MELO_DEBUG("Number of bus managers: %lu", busManagers.size());
    MELO_DEBUG("Number of RokubiminiSetups: %lu", setup->rokubiminis_.size());
  }
}

bool checkAndLoadBusSetupFromFile(const rokubimini::setup::SetupPtr& setup, const std::string& setupFile,
                                  vector<unique_ptr<RokubiminiBusManager>>& busManagers)
{
  // If loadSetup() without argument is called, the setupFile is empty.
  // In this case, the default setup is used.
  if (!setupFile.empty())
  {
    MELO_INFO("Loading setup file");
    try
    {
      loadBusSetupFromFile(setup, setupFile, busManagers);
      // setup->fromFile(setupFile);
    }
    catch (const message_logger::log::melo_fatal& exception)
    {
      MELO_ERROR_STREAM("Caught an exception while loading the setup: " << exception.what());
      return false;
    }
  }
  return true;
}
rokubimini::setup::SetupPtr createSetup()
{
  return rokubimini::setup::SetupPtr(new rokubimini::setup::Setup());
}
rokubimini::setup::SetupPtr loadBusSetup(const std::string& configurationFile,
                                         vector<unique_ptr<RokubiminiBusManager>>& busManagers)
{
  auto setup = createSetup();
  if (!checkAndLoadBusSetupFromFile(setup, configurationFile, busManagers))
  {
    MELO_ERROR("Could not load Bus Setup from file!");
    return nullptr;
  }
  return setup;
}

}  // namespace rokubimini