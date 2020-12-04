// rokubimini
#include <rokubimini_ethercat/setup/RokubiminiEthercat.hpp>

#include <message_logger/message_logger.hpp>

namespace rokubimini
{
namespace ethercat
{
namespace setup
{
void RokubiminiEthercat::fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile)
{
  Rokubimini::fromFile(yamlNode, setupFile);
  if (yamlNode.hasKey("ethercat_bus"))
  {
    ethercatBus_ = yamlNode["ethercat_bus"].as<std::string>();
  }
  if (yamlNode.hasKey("ethercat_address"))
  {
    const auto ethercat_address = yamlNode["ethercat_address"].as<uint32_t>();
    if (ethercat_address == 0)
    {
      throw message_logger::log::melo_fatal("The EtherCAT address is 0.");
    }
    ethercatAddress_ = ethercat_address;
  }
  if (yamlNode.hasKey("ethercat_pdo_type"))
  {
    const std::string pdo_type = yamlNode["ethercat_pdo_type"].as<std::string>();
    if (pdo_type == "A")
    {
      ethercatPdoTypeEnum_ = PdoTypeEnum::A;
    }
    else if (pdo_type == "B")
    {
      ethercatPdoTypeEnum_ = PdoTypeEnum::B;
    }
    else if (pdo_type == "C")
    {
      ethercatPdoTypeEnum_ = PdoTypeEnum::C;
    }
    else if (pdo_type == "Z")
    {
      ethercatPdoTypeEnum_ = PdoTypeEnum::Z;
    }
    else if (pdo_type == "EXTIMU")
    {
      ethercatPdoTypeEnum_ = PdoTypeEnum::EXTIMU;
    }
    else
    {
      throw message_logger::log::melo_fatal("EtherCAT PDO Type '" + pdo_type + "' could not be parsed.");
    }
  }
}

}  // namespace setup
}  // namespace ethercat
}  // namespace rokubimini
