#include <rokubimini/setup/Rokubimini.hpp>

#include <message_logger/message_logger.hpp>

namespace rokubimini
{
namespace setup
{
void Rokubimini::fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile)
{
  if (yamlNode.hasKey("name"))
  {
    name_ = yamlNode["name"].as<std::string>();
  }
  if (yamlNode.hasKey("product_code"))
  {
    productCode_ = yamlNode["product_code"].as<std::uint32_t>();
  }
  if (yamlNode.hasKey("configuration_file"))
  {
    std::string configuration_file = yamlNode["configuration_file"].as<std::string>();
    if (configuration_file.empty())
    {
      throw message_logger::log::melo_fatal("The path to the configuration file is empty.");
    }
    else if (configuration_file.front() == '/')
    {
      // Path to the configuration file is absolute, we can use it as is.
    }
    else if (configuration_file.front() == '~')
    {
      // Path to the configuration file is absolute, we need to replace '~' with the home directory.
      const char* home_directory = getenv("HOME");
      if (home_directory == nullptr)
      {
        throw message_logger::log::melo_fatal("Environment variable 'HOME' could not be evaluated.");
      }
      configuration_file.erase(configuration_file.begin());
      configuration_file = home_directory + configuration_file;
    }
    else
    {
      // Path to the configuration file is relative, we need to append it to the path of the setup file.
      configuration_file = setupFile.substr(0, setupFile.find_last_of('/') + 1) + configuration_file;
    }

    configuration_.fromFile(configuration_file);
  }
}

}  // namespace setup
}  // namespace rokubimini