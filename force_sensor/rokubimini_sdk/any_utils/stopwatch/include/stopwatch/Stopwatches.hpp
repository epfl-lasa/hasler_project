#pragma once

// std
#include <map>

// stopwatch
#include "stopwatch/Stopwatch.hpp"

namespace stopwatch
{
/*!
 * Stopwatches class.
 *
 * Contains a static unordered map containing stopwatch instances making
 * them accessible from anywhere. Each stopwatch is identified by a unique name.
 */
class Stopwatches
{
protected:
  /*!
   * Container class containing all stopwatches created using getOrCreate(..).
   * Use a map instead of an unordered_map for alphabetic printing.
   */
  using StopwatchContainer = std::map<std::string, StopwatchPtr>;

  //! Container class containing all stopwatches created using getOrCreate(..).
  static StopwatchContainer stopwatchContainer_;

public:
  /*!
   * Constructor.
   * Stopwatches only contains static methods and therefore is not intended
   * to be instantiated.
   */
  Stopwatches() = delete;

  /*!
   * Destructor.
   */
  ~Stopwatches() = delete;

  /*!
   * Get a stopwatch or create it if it does not exist yet.
   * Static method, can be accessed from anywhere. The stopwatch
   * is not started automatically on creation. Good practice is to
   * save the pointer to a member variable of your class to avoid
   * calling this function in every execution.
   * @param name Name of the stopwatch.
   * @return Pointer to the stopwatch.
   */
  static StopwatchPtr getOrCreate(const std::string& name);

  /*!
   * Clear all stopwatches, resetting them to the construction state.
   * Static method, so it can be accessed from anywhere.
   */
  static void clearAll();

  /*!
   * Print the statistics from all stopwatches.
   * Static method, so it can be accessed from anywhere.
   */
  static void printAll();

  /*!
   * Write the statistics from all stopwatches to a CSV file.
   * Static method, so it can be accessed from anywhere.
   * @param folderPath        Path to the folder, can contain '~' for the home directory.
   * @param fileName          Name of the file, without date or ending.
   * @param addDateToFileName If true, the file name will be extended by the current date (including time).
   * @return True if successful.
   */
  static bool writeAllToCsvFile(const std::string& folderPath, const std::string& fileName,
                                const bool addDateToFileName);
};

}  // namespace stopwatch
