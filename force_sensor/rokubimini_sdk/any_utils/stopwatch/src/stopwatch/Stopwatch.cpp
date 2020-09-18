// std
#include <fstream>
#include <iomanip>
#include <sstream>

// stopwatch
#include "stopwatch/Stopwatch.hpp"

namespace stopwatch
{
Stopwatch::Stopwatch(std::string name) : name_(std::move(name))
{
}

void Stopwatch::start()
{
  if (isRunning_)
  {
    return;
  }

  isRunning_ = true;
  intermediateMeasurement_ = 0.0;
  startingTime_ = Clock::now();
}

void Stopwatch::pause()
{
  if (!isRunning_)
  {
    return;
  }

  const TimePoint now = Clock::now();
  const Duration elapsedSeconds = now - startingTime_;
  intermediateMeasurement_ += elapsedSeconds.count();
  startingTime_ = now;
  isRunning_ = false;
}

void Stopwatch::resume()
{
  if (isRunning_)
  {
    return;
  }

  isRunning_ = true;
  startingTime_ = Clock::now();
}

void Stopwatch::stop()
{
  if (!isRunning_)
  {
    return;
  }

  const Duration elapsedSeconds = Clock::now() - startingTime_;
  statistics_.addMeasurement(intermediateMeasurement_ + elapsedSeconds.count());

  isRunning_ = false;
}

void Stopwatch::clear()
{
  isRunning_ = false;
  intermediateMeasurement_ = 0.0;
  statistics_.clear();
}

void Stopwatch::print() const
{
  std::cout << asString() << std::endl;
}

std::string Stopwatch::asString() const
{
  std::stringstream ss;
  ss << "name: " << getName() << ", " << statistics_.asString();
  return ss.str();
}

std::string Stopwatch::getTitleAsCsvString()
{
  return std::string("name, ") + Statistics::getTitleAsCsvString();
}

std::string Stopwatch::getDataAsCsvString() const
{
  std::stringstream ss;
  ss << getName() << ", " << getStatistics().getDataAsCsvString();
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const Stopwatch& stopwatch)
{
  out << stopwatch.asString();
  return out;
}

}  // namespace stopwatch
