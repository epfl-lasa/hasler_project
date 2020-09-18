// rokubimini
#include "rokubimini/setup/Setup.hpp"

namespace rokubimini
{
namespace setup
{
Setup::Setup()
{
  // Create the default setup.
  rokubiminis_.push_back(this->createRokubimini());
}

void Setup::fromFile(const std::string& setupFile)
{
  // Load the setup from the file.
  yaml_tools::YamlNode yaml_node = yaml_tools::YamlNode::fromFile(setupFile);

  if (yaml_node.hasKey("rokubiminis"))
  {
    // Clear the rokubiminis first.
    rokubiminis_.clear();

    const yaml_tools::YamlNode rokubiminis = yaml_node["rokubiminis"];
    for (const auto& i : rokubiminis)
    {
      auto rokubimini = this->createRokubimini();
      rokubimini->fromFile(i, setupFile);
      rokubiminis_.push_back(rokubimini);
    }
  }
}

std::shared_ptr<Rokubimini> Setup::createRokubimini() const
{
  return std::make_shared<Rokubimini>();
}

}  // namespace setup
}  // namespace rokubimini
