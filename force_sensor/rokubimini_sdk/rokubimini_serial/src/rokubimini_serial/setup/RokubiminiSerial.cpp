// rokubimini
#include <rokubimini_serial/setup/RokubiminiSerial.hpp>

#include <message_logger/message_logger.hpp>

namespace rokubimini
{
namespace serial
{
namespace setup
{
void RokubiminiSerial::fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile)
{
  Rokubimini::fromFile(yamlNode, setupFile);
  if (yamlNode.hasKey("port"))
  {
    serialPort_ = yamlNode["port"].as<std::string>();
  }
  if (yamlNode.hasKey("baud_rate"))
  {
    baudRate_ = yamlNode["baud_rate"].as<uint32_t>();
  }
  // if (yamlNode.hasKey("version"))
  // {
  //   version_ = yamlNode["version"].as<std::string>();
  // }
  // if (yamlNode.hasKey("dev_pid"))
  // {
  //   devPid_ = yamlNode["dev_pid"].as<std::uint32_t>();
  // }
}

}  // namespace setup
}  // namespace serial
}  // namespace rokubimini
