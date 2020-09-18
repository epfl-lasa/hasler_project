// std
#include <fstream>
#include <iomanip>
#include <sstream>

// stopwatch
#include "stopwatch/Stopwatches.hpp"

namespace stopwatch
{
Stopwatches::StopwatchContainer Stopwatches::stopwatchContainer_;

StopwatchPtr Stopwatches::getOrCreate(const std::string& name)
{
  // Try to find the stopwatch.
  auto stopwatch = stopwatchContainer_.find(name);
  if (stopwatch == stopwatchContainer_.end())
  {
    // Create a new stopwatch.
    return stopwatchContainer_.insert({ name, std::make_shared<Stopwatch>(name) }).first->second;
  }
  else
  {
    // Return the existing stopwatch.
    return stopwatch->second;
  }
}

void Stopwatches::clearAll()
{
  for (auto& stopwatch : stopwatchContainer_)
  {
    stopwatch.second->clear();
  }
}

void Stopwatches::printAll()
{
  for (const auto& stopwatch : stopwatchContainer_)
  {
    stopwatch.second->print();
  }
}

bool Stopwatches::writeAllToCsvFile(const std::string& folderPath, const std::string& fileName,
                                    const bool addDateToFileName)
{
  // Derive the folder path.
  std::string folderPathAbsolute = folderPath;
  if (folderPathAbsolute.front() == '~')
  {
    const char* homeDirectory = getenv("HOME");
    if (homeDirectory == nullptr)
    {
      return false;
    }
    folderPathAbsolute.erase(folderPathAbsolute.begin());
    folderPathAbsolute = homeDirectory + folderPathAbsolute;
  }

  // Derive the file path.
  std::stringstream filePath;
  filePath << folderPathAbsolute << "/" << fileName;
  if (addDateToFileName)
  {
    auto time = std::time(nullptr);
    auto timepoint = *std::localtime(&time);
    filePath << "_" << std::put_time(&timepoint, "%Y-%m-%d_%H-%M-%S");
  }
  filePath << ".csv";

  // Open the file.
  std::ofstream file;
  file.open(filePath.str());
  if (file.bad())
  {
    return false;
  }

  // Write to the file.
  file << Stopwatch::getTitleAsCsvString() << std::endl;
  for (auto& stopwatch : stopwatchContainer_)
  {
    file << stopwatch.second->getDataAsCsvString() << std::endl;
  }

  // Close the file.
  file.close();
  return true;
}

}  // namespace stopwatch
