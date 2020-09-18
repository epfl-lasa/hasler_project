#pragma once

#include <cstdint>

#include <yaml_tools/YamlNode.hpp>

namespace rokubimini
{
namespace configuration
{
/**
 * @class ForceTorqueFilter
 *
 * @brief Class holding the force-torque filter settings.
 *
*/
class ForceTorqueFilter
{
public:
  /**
   * @fn ForceTorqueFilter()
   *
   * @brief Default constructor.
   *
  */
  ForceTorqueFilter() = default;

  /**
   * @fn ForceTorqueFilter(const uint16_t sincFilterSize, const uint8_t chopEnable, const uint8_t skipEnable, const
   * uint8_t fastEnable)
   *
   * @brief Custom constructor which accepts custom settings and creates the object.
   *
   * @param sincFilterSize The size of the sinc filter.
   * @param chopEnable The chopEnable filter flag.
   * @param skipEnable The skipEnable filter flag.
   * @param fastEnable The fastEnable filter flag.
  */
  ForceTorqueFilter(const uint16_t sincFilterSize, const uint8_t chopEnable, const uint8_t skipEnable,
                    const uint8_t fastEnable);

  ~ForceTorqueFilter() = default;

  /**
   * @fn void fromFile(const yaml_tools::YamlNode &yamlNode)
   *
   * @brief Loads the ForceTorque Filter settings from file.
   *
   * @param yamlNode The yaml node to parse.
  */
  void fromFile(const yaml_tools::YamlNode& yamlNode);

  /**
   * @fn uint16_t getSincFilterSize() const
   *
   * @brief Gets the \a sincFilterSize variable.
   * @return The \a sincFilterSize value.
   *
  */
  uint16_t getSincFilterSize() const
  {
    return sincFilterSize_;
  }

  /**
   * @fn void setSincFilterSize(const uint16_t sincFilterSize)
   *
   * @brief Sets the \a sincFilterSize variable.
   * @param sincFilterSize The value to set.
   *
  */
  void setSincFilterSize(const uint16_t sincFilterSize)
  {
    sincFilterSize_ = sincFilterSize;
  }

  /**
   * @fn uint8_t getChopEnable() const
   *
   * @brief Gets the \a chopEnable variable.
   * @return The \a chopEnable value.
   *
  */
  uint8_t getChopEnable() const
  {
    return chopEnable_;
  }

  /**
   * @fn void setChopEnable(const uint8_t chopEnable)
   *
   * @brief Sets the \a chopEnable variable.
   * @param chopEnable The value to set.
   *
  */
  void setChopEnable(const uint8_t chopEnable)
  {
    chopEnable_ = chopEnable;
  }

  /**
   * @fn uint8_t getSkipEnable() const
   *
   * @brief Gets the \a skipEnable variable.
   * @return The \a skipEnable value.
   *
  */
  uint8_t getSkipEnable() const
  {
    return skipEnable_;
  }

  /**
   * @fn void setSkipEnable(const uint8_t skipEnable)
   *
   * @brief Sets the \a skipEnable variable.
   * @param skipEnable The value to set.
   *
  */
  void setSkipEnable(const uint8_t skipEnable)
  {
    skipEnable_ = skipEnable;
  }

  /**
   * @fn uint8_t getFastEnable() const
   *
   * @brief Gets the \a fastEnable variable.
   * @return The \a fastEnable value.
   *
  */
  uint8_t getFastEnable() const
  {
    return fastEnable_;
  }

  /**
   * @fn void setFastEnable(const uint8_t fastEnable)
   *
   * @brief Sets the \a fastEnable variable.
   * @param fastEnable The value to set.
   *
  */
  void setFastEnable(const uint8_t fastEnable)
  {
    fastEnable_ = fastEnable;
  }

  /**
   * @fn void print() const
   *
   * @brief Prints the existing Filter settings.
   *
  */
  void print() const;

private:
  /**
   * @var uint16_t sincFilterSize_
   *
   * @brief The sincFilterSize variable.
   *
  */
  uint16_t sincFilterSize_;

  /**
   * @var uint8_t chopEnable_
   *
   * @brief The chopEnable flag.
   *
  */
  uint8_t chopEnable_;

  /**
   * @var uint8_t skipEnable_
   *
   * @brief The skipEnable flag.
   *
  */
  uint8_t skipEnable_;

  /**
   * @var uint8_t fastEnable_
   *
   * @brief The fastEnable flag.
   *
  */
  uint8_t fastEnable_;
};

}  // namespace configuration
}  // namespace rokubimini