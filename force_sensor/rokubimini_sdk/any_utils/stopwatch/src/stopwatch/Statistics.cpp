// std
#include <sstream>

// stopwatch
#include "stopwatch/Statistics.hpp"

namespace stopwatch
{
void Statistics::addMeasurement(const double measurement)
{
  lastMeasurement_ = measurement;
  numMeasurements_++;
  const double delta = measurement - mean_;
  mean_ += delta / numMeasurements_;
  const double delta2 = measurement - mean_;
  m2_ += delta * delta2;
  min_ = std::min(min_, measurement);
  max_ = std::max(max_, measurement);
}

void Statistics::clear()
{
  lastMeasurement_ = nan_;
  numMeasurements_ = 0;
  mean_ = 0.0;
  m2_ = 0.0;
  min_ = inf_;
  max_ = 0.0;
}

std::string Statistics::asString() const
{
  std::stringstream ss;
  ss << "num meas: " << getNumMeasurements() << ", "
     << "mean: " << getMean() << ", "
     << "var: " << getVar() << ", "
     << "std dev: " << getStdDev() << ", "
     << "min: " << getMin() << ", "
     << "max: " << getMax();
  return ss.str();
}

std::string Statistics::getTitleAsCsvString()
{
  return "num meas, mean, var, std dev, min, max";
}

std::string Statistics::getDataAsCsvString() const
{
  std::stringstream ss;
  ss << getNumMeasurements() << ", " << getMean() << ", " << getVar() << ", " << getStdDev() << ", " << getMin() << ", "
     << getMax();
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const Statistics& statistics)
{
  out << statistics.asString();
  return out;
}

}  // namespace stopwatch
