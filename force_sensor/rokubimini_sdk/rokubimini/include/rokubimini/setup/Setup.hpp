#pragma once

// std
#include <memory>
#include <vector>

// yaml tools
#include <yaml_tools/YamlNode.hpp>
#include <yaml_tools/ConstIterator.hpp>

// rokubimini
#include "rokubimini/configuration/Configuration.hpp"
#include "rokubimini/setup/Rokubimini.hpp"

namespace rokubimini
{
namespace setup
{
/**
 * @class Setup
 *
 * @brief Class represting the setup loaded from a setup file.
 *
*/
class Setup
{
public:
  /**
   * @fn Setup()
   *
   * @brief Default constructor. Initializes the list by adding one Rokubimini Setup.
   *
  */
  Setup();
  virtual ~Setup() = default;

  /**
   * @fn void fromFile(const std::string &setupFile)
   *
   * @brief Loads the Setup found in the setup file.
   *
   * @param setupFile The name of the setup file.
  */
  void fromFile(const std::string& setupFile);

  /**
   * @fn std::shared_ptr<Rokubimini> createRokubimini() const
   *
   * @brief Creates a Rokubimini Setup instance.
   * @return The pointer to the newly created Rokubimini Setup instance.
   *
  */
  virtual std::shared_ptr<Rokubimini> createRokubimini() const;

  /**
   * @var std::vector<std::shared_ptr<Rokubimini>> rokubiminis_
   *
   * @brief The list with all the Rokubimini Setups.
   *
  */
  std::vector<std::shared_ptr<Rokubimini>> rokubiminis_;
};

using SetupPtr = std::shared_ptr<Setup>;

}  // namespace setup
}  // namespace rokubimini
