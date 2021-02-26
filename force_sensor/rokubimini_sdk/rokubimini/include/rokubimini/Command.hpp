#pragma once

#include <any_measurements/Time.hpp>
#include <any_measurements/Wrench.hpp>

namespace rokubimini
{
/**
 * @class Command
 *
 * @brief Class representing the commands sent to the
 * rokubi mini devices.
 *
 * todo{clarification of bools}
*/
class Command
{
public:
  using WrenchType = any_measurements::Wrench;

  /**
   * @fn Command()
   *
   * @brief Default constructor.
   *
  */
  Command() = default;
  virtual ~Command() = default;

  /**
   * @fn bool isSetAsZeroTare() const
   *
   * @brief Gets the \a setAsZeroTare flag.
   * @return The \a setAsZeroTare value.
   *
  */
  bool isSetAsZeroTare() const
  {
    return setAsZeroTare_;
  }

  /**
   * @fn void setAsZeroTare(bool setAsZeroTare)
   *
   * @brief Sets the \a setAsZeroTare variable.
   * @param setAsZeroTare The value to set.
   *
  */
  void setAsZeroTare(bool setAsZeroTare)
  {
    setAsZeroTare_ = setAsZeroTare;
  }

  /**
   * @fn bool isResetTareLoad() const
   *
   * @brief Gets the \a resetTareLoad flag.
   * @return The \a resetTareLoad value.
   *
  */
  bool isResetTareLoad() const
  {
    return resetTareLoad_;
  }

  /**
   * @fn void setResetTareLoad(bool resetTareLoad)
   *
   * @brief Sets the \a resetTareLoad variable.
   * @param resetTareLoad The value to set.
   *
  */
  void setResetTareLoad(bool resetTareLoad)
  {
    resetTareLoad_ = resetTareLoad;
  }

  /**
   * @fn const WrenchType &getZeroTare() const
   *
   * @brief Gets the \a zeroTare variable.
   * @return The \a zeroTare value.
   *
  */
  const WrenchType& getZeroTare() const
  {
    return zeroTare_;
  }

  /**
   * @fn WrenchType &getZeroTare()
   *
   * @brief Non-const version of \a getZeroTare() const. Gets the \a zeroTare variable.
   * @return The \a zeroTare value.
   *
  */
  WrenchType& getZeroTare()
  {
    return zeroTare_;
  }

  /**
   * @fn void setStamp(const any_measurements::Time &stamp)
   *
   * @brief Sets the \a stamp variable.
   * @param stamp The value to set.
   *
  */
  void setStamp(const any_measurements::Time& stamp)
  {
    stamp_ = stamp;
  }

  /**
   * @fn const any_measurements::Time &getStamp() const
   *
   * @brief Gets the \a stamp variable.
   * @return The \a stamp value.
   *
  */
  const any_measurements::Time& getStamp() const
  {
    return stamp_;
  }

protected:
  /**
   * @var any_measurements::Time stamp_
   *
   * @brief The stamp variable.
   *
  */
  any_measurements::Time stamp_;

  /**
   * @var bool setAsZeroTare_
   *
   * @brief The setAsZeroTare flag.
   *
  */
  bool setAsZeroTare_{ false };

  /**
   * @var bool resetTareLoad_
   *
   * @brief The resetTareLoad flag.
   *
  */
  bool resetTareLoad_{ false };

  /**
   * @var WrenchType zeroTare_
   *
   * @brief The zeroTare variable.
   *
  */
  WrenchType zeroTare_;
};

}  // namespace rokubimini
