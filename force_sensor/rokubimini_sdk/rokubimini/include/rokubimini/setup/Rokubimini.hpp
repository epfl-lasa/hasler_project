#pragma once

// std
#include <memory>
#include <string>

// yaml tools
#include <yaml_tools/YamlNode.hpp>

// rokubimini
#include "rokubimini/configuration/Configuration.hpp"

namespace rokubimini
{
namespace setup
{
/**
 * @class Rokubimini
 *
 * @brief Class represting an abstract Rokubimini Setup.
 * It's used for holding all the setup info relevant to a Rokubimini.
 *
*/
class Rokubimini
{
public:
  /**
   * @fn Setup()
   *
   * @brief Default constructor.
   *
  */
  Rokubimini() = default;
  virtual ~Rokubimini() = default;

  /**
   * @fn void fromFile(const yaml_tools::YamlNode &yamlNode, const std::string &setupFile)
   *
   * @brief Loads the Rokubimini Setup found in the setup file.
   *
   * @param yamlNode The yaml node to be parsed, located in the setup file.
   * @param setupFile The name of the setup file.
  */
  virtual void fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile);

public:
  /**
   * @var std::string name_
   *
   * @brief The name of the Rokubimini Setup.
   *
  */
  std::string name_{ "rokubimini" };

  /**
   * @var std::uint32_t productCode_
   *
   * @brief The product code of the Rokubimini Setup.
   *
  */
  std::uint32_t productCode_{ 0 };

  /**
   * @var configuration::Configuration configuration_
   *
   * @brief The configuration of the Rokubimini Setup.
   *
  */
  configuration::Configuration configuration_;
};

}  // namespace setup
}  // namespace rokubimini
