#pragma once

// std
#include <cstdint>

// rokubimini
#include <rokubimini/setup/Rokubimini.hpp>

// rokubimini ethercat
#include "rokubimini_ethercat/PdoTypeEnum.hpp"

namespace rokubimini
{
namespace ethercat
{
namespace setup
{
using Rokubimini = rokubimini::setup::Rokubimini;

/**
 * @class RokubiminiEthercat
 *
 * @brief The Rokubimini RokubiminiEthercat Setup class.
 *
 * Inherits from the Rokubimini Setup class. It's used for
 * parsing a setup file and creating a Rokubimini Ethercat Setup
 * from it, by parsing specific ethercat attributes from the file
 * (ethercat bus, ethercat address, etc.).
*/

class RokubiminiEthercat : public rokubimini::setup::Rokubimini
{
public:
  /**
   * @fn RokubiminiEthercat()
   *
   * @brief Default constructor of a Rokubimini Ethercat Setup.
   *
  */

  RokubiminiEthercat() = default;
  ~RokubiminiEthercat() override = default;

  /**
   * @fn void fromFile(const yaml_tools::YamlNode &yamlNode, const std::string &setupFile)
   *
   * @brief Parses the @param setupFile and gets the ethercat-specific information (ethercat bus, ethercat address,
   * etc.) from it.
   *
   * @param yamlNode The specific yaml node to parse for ethercat
   * specifics.
   * @param setupFile The setup file.
  */

  void fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile) override;

  /**
   * @var std::string ethercatBus_
   *
   * @brief The ethercat bus name.
  */

  std::string ethercatBus_{ "eth0" };

  /**
   * @var uint32_t ethercatAddress_
   *
   * @brief The ethercat address.
  */

  uint32_t ethercatAddress_{ 1 };

  /**
   * @var PdoTypeEnum ethercatPdoTypeEnum_
   *
   * @brief The PDO type.
  */

  PdoTypeEnum ethercatPdoTypeEnum_{ PdoTypeEnum::A };
};

}  // namespace setup
}  // namespace ethercat
}  // namespace rokubimini
