#pragma once

#include <any_measurements/Imu.hpp>
#include <any_measurements/Wrench.hpp>

#include <rokubimini/Statusword.hpp>

namespace rokubimini
{
/**
 * @class Reading
 *
 * @brief Class representing the readings received from the
 * rokubi mini devices.
 *
*/
class Reading
{
public:
  using ImuType = any_measurements::Imu;
  using WrenchType = any_measurements::Wrench;

  /**
   * @fn Command()
   *
   * @brief Default constructor.
   *
  */
  Reading() = default;
  virtual ~Reading() = default;

  /**
   * @fn const ImuType &getImu() const
   *
   * @brief Gets the \a imu variable.
   * @return The \a imu values.
   *
  */
  const ImuType& getImu() const
  {
    return imu_;
  }

  /**
   * @fn ImuType &getImu()
   *
   * @brief Non-const version of getImu() const. Gets the \a imu variable.
   * @return The \a imu values.
   *
  */
  ImuType& getImu()
  {
    return imu_;
  }

  /**
   * @fn const WrenchType &getWrench() const
   *
   * @brief Gets the \a wrench variable.
   * @return The \a wrench values.
   *
  */

  const WrenchType& getWrench() const
  {
    return wrench_;
  }

  /**
   * @fn WrenchType &getWrench()
   *
   * @brief Non-const version of getWrench() const. Gets the \a wrench variable.
   * @return The \a wrench values.
   *
  */
  WrenchType& getWrench()
  {
    return wrench_;
  }

  /**
   * @fn const ImuType &getExternalImu() const
   *
   * @brief Gets the \a externalImu variable.
   * @return The \a externalImu values.
   *
  */
  const ImuType& getExternalImu() const
  {
    return externalImu_;
  }

  /**
   * @fn ImuType &getExternalImu()
   *
   * @brief Non-const version of getExternalImu() const. Gets the \a externalImu variable.
   * @return The \a externalImu values.
   *
  */
  ImuType& getExternalImu()
  {
    return externalImu_;
  }

  /**
   * @fn const Statusword &getStatusword() const
   *
   * @brief Gets the \a statusword variable.
   * @return The \a statusword value.
   *
  */
  const Statusword& getStatusword() const
  {
    return statusword_;
  }

  /**
   * @fn void setStatusword(const Statusword &statusword)
   *
   * @brief Sets the \a statusword variable.
   * @param statusword The value to set.
   *
  */
  void setStatusword(const Statusword& statusword)
  {
    statusword_ = statusword;
  }

  /**
   * @fn bool isForceTorqueSaturated() const
   *
   * @brief Gets the \a isForceTorqueSaturated flag.
   * @return The \a isForceTorqueSaturated value.
   *
  */
  bool isForceTorqueSaturated() const
  {
    return isForceTorqueSaturated_;
  }

  /**
   * @fn void setForceTorqueSaturated(const bool isForceTorqueSaturated)
   *
   * @brief Sets the \a isForceTorqueSaturated variable.
   * @param isForceTorqueSaturated The value to set.
   *
  */
  void setForceTorqueSaturated(const bool isForceTorqueSaturated)
  {
    isForceTorqueSaturated_ = isForceTorqueSaturated;
  }

  /**
   * @fn float getTemperature() const
   *
   * @brief Gets the \a temperature flag.
   * @return The \a temperature value.
   *
  */
  float getTemperature() const
  {
    return temperature_;
  }

  /**
   * @fn void setTemperature(const float temperature)
   *
   * @brief Sets the \a temperature variable.
   * @param temperature The value to set.
   *
  */
  void setTemperature(const float temperature)
  {
    temperature_ = temperature;
  }

protected:
  /**
   * @var ImuType imu_
   *
   * @brief The imu variable.
   *
  */
  ImuType imu_;

  /**
   * @var WrenchType wrench_
   *
   * @brief The wrench variable.
   *
  */
  WrenchType wrench_;

  /**
   * @var ImuType externalImu_
   *
   * @brief The externalImu variable.
   *
  */
  ImuType externalImu_;

  /**
   * @var bool isForceTorqueSaturated_
   *
   * @brief The isForceTorqueSaturated variable.
   *
  */
  bool isForceTorqueSaturated_{ false };

  /**
   * @var float temperature_
   *
   * @brief The temperature variable.
   *
  */
  float temperature_;

  /**
   * @var float statusword_
   *
   * @brief The statusword variable.
   *
  */
  Statusword statusword_;
};

}  // namespace rokubimini
