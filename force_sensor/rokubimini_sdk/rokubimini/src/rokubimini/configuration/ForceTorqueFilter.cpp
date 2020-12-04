#include <rokubimini/configuration/ForceTorqueFilter.hpp>

namespace rokubimini
{
namespace configuration
{
ForceTorqueFilter::ForceTorqueFilter(const uint16_t sincFilterSize, const uint8_t chopEnable, const uint8_t skipEnable,
                                     const uint8_t fastEnable)
  : sincFilterSize_(sincFilterSize), chopEnable_(chopEnable), skipEnable_(skipEnable), fastEnable_(fastEnable)
{
}

void ForceTorqueFilter::fromFile(const yaml_tools::YamlNode& yamlNode)
{
  sincFilterSize_ = static_cast<uint16_t>(yamlNode["force_torque_filter"]["sinc_filter_size"].as<int>());
  chopEnable_ = static_cast<uint8_t>(yamlNode["force_torque_filter"]["chop_enable"].as<bool>());
  skipEnable_ = static_cast<uint8_t>(yamlNode["force_torque_filter"]["fir_disable"].as<bool>());
  fastEnable_ = static_cast<uint8_t>(yamlNode["force_torque_filter"]["fast_enable"].as<bool>());
}

void ForceTorqueFilter::print() const
{
  MELO_INFO_STREAM("sinc_filter_size_: " << static_cast<unsigned int>(sincFilterSize_));
  MELO_INFO_STREAM("chopEnable_: " << static_cast<unsigned int>(chopEnable_));
  MELO_INFO_STREAM("skipEnable_: " << static_cast<unsigned int>(skipEnable_));
  MELO_INFO_STREAM("fastEnable_: " << static_cast<unsigned int>(fastEnable_));
}

}  // namespace configuration
}  // namespace rokubimini