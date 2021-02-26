#pragma once

// std
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

// stopwatch
#include "stopwatch/Statistics.hpp"

namespace stopwatch
{
/*!
 * Stopwatch class.
 *
 * A stopwatch can be used for measuring the execution times.
 *
 * Features:
 *   - Statistics, including mean, variance, standard deviation, min and max
 *   - Pause and resume
 *   - Write statistics to CSV file
 */
class Stopwatch
{
protected:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Duration = std::chrono::duration<double>;

  const std::string name_;

  bool isRunning_ = false;
  TimePoint startingTime_;
  double intermediateMeasurement_ = 0.0;

  Statistics statistics_;

public:
  /*!
   * Constructor.
   * @param name Name of the stopwatch.
   */
  explicit Stopwatch(std::string name);

  /*!
   * Destructor.
   */
  ~Stopwatch() = default;

  /*!
   * Get the name of the stopwatch.
   * @return Name of the stopwatch.
   */
  const std::string& getName() const
  {
    return name_;
  }

  /*!
   * Check if the stopwatch is running.
   * A stopwatch is running if it has been started, but not paused.
   * @return True if the stopwatch is running.
   */
  bool isRunning() const
  {
    return isRunning_;
  }

  /*!
   * Start the stopwatch.
   */
  void start();

  /*!
   * Pause the stopwatch.
   * Does not take a measurement.
   */
  void pause();

  /*!
   * Resume the stopwatch.
   */
  void resume();

  /*!
   * Stop the stopwatch and take a measurement.
   */
  void stop();

  /*!
   * Get the statistics as reference.
   * @return Statistics as reference.
   */
  Statistics& getStatistics()
  {
    return statistics_;
  }

  /*!
   * Get the statistics.
   * @return Statistics.
   */
  const Statistics& getStatistics() const
  {
    return statistics_;
  }

  /*!
   * Clear the stopwatch, resetting it to the construction state.
   */
  void clear();

  /*!
   * Print the stopwatch.
   */
  void print() const;

  /*!
   * Get the name and statistics as string.
   * @return Name and statistics as string.
   */
  std::string asString() const;

  /*!
   * Get the title of the stopwatch as CSV formatted string.
   * @return Title of the stopwatch.
   */
  static std::string getTitleAsCsvString();

  /*!
   * Get the data of the stopwatch as CSV formatted string.
   * @return Data of the stopwatch.
   */
  std::string getDataAsCsvString() const;
};

//! ostream overload for a stopwatch object.
std::ostream& operator<<(std::ostream& out, const Stopwatch& stopwatch);

//! Typedef for convenience.
using StopwatchPtr = std::shared_ptr<Stopwatch>;

}  // namespace stopwatch
