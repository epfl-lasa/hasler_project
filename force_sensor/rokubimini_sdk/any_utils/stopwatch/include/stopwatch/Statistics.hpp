#pragma once

// std
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

namespace stopwatch
{
/*!
 * Statistics class.
 *
 * Contains stopwatch statistics.
 */
class Statistics
{
protected:
  const double nan_ = std::numeric_limits<double>::quiet_NaN();
  const double inf_ = std::numeric_limits<double>::infinity();

  double lastMeasurement_ = nan_;
  uint64_t numMeasurements_ = 0;
  double mean_ = 0.0;
  double m2_ = 0.0;
  double min_ = inf_;
  double max_ = 0.0;

public:
  /*!
   * Constructor.
   */
  Statistics() = default;

  /*!
   * Destructor.
   */
  ~Statistics() = default;

  /*!
   * Add a measurement to the statistics.
   * @param measurement Measurement to add.
   */
  void addMeasurement(const double measurement);

  /*!
   * Get the last measurement which has been taken.
   * @return Last measurement.
   */
  double getLastMeasurement() const
  {
    return lastMeasurement_;
  }

  /*!
   * Get the number of measurements which have been taken.
   * @return Number of measurements.
   */
  uint64_t getNumMeasurements() const
  {
    return numMeasurements_;
  }

  /*!
   * Get the mean of the measurements.
   * @return Mean of the measurements.
   */
  double getMean() const
  {
    return (numMeasurements_ == 0) ? nan_ : mean_;
  }

  /*!
   * Get the variance of the measurements.
   * @return Variance of the measurements.
   */
  double getVar() const
  {
    return (numMeasurements_ <= 1) ? nan_ : m2_ / (numMeasurements_ - 1);
  }

  /*!
   * Get the standard deviation of the measurements.
   * @return Standard deviation of the measurements.
   */
  double getStdDev() const
  {
    return std::sqrt(getVar());
  }

  /*!
   * Get the minimal measurement.
   * @return Minimal measurement.
   */
  double getMin() const
  {
    return (numMeasurements_ == 0) ? nan_ : min_;
  }

  /*!
   * Get the maximal measurement.
   * @return Maximal measurement.
   */
  double getMax() const
  {
    return (numMeasurements_ == 0) ? nan_ : max_;
  }

  /*!
   * Clear the statistics, resetting it to the construction state.
   */
  void clear();

  /*!
   * Get the statistics as string.
   * @return Statistics as string.
   */
  std::string asString() const;

  /*!
   * Get the title of the statistics as CSV formatted string.
   * @return Title of the statistics.
   */
  static std::string getTitleAsCsvString();

  /*!
   * Get the data of the statistics as CSV formatted string.
   * @return Data of the statistics.
   */
  std::string getDataAsCsvString() const;
};

//! ostream overload for a statistics object.
std::ostream& operator<<(std::ostream& out, const Statistics& statistics);

}  // namespace stopwatch
