// soem_interface
#include "rokubimini_ethercat/soem_interface/common/Macros.hpp"

namespace soem_interface
{
namespace common
{
std::mutex MessageLog::logMutex_;
MessageLog::Log MessageLog::log_;

void MessageLog::insertMessage(message_logger::log::levels::Level level, const std::string& message)
{
  std::lock_guard<std::mutex> lock(logMutex_);
  log_.push_back({ level, message });
  if (log_.size() > MAX_LOG_SIZE)
  {
    log_.pop_front();
  }
}

MessageLog::Log MessageLog::getLog()
{
  std::lock_guard<std::mutex> lock(logMutex_);
  return log_;
}

void MessageLog::clearLog()
{
  std::lock_guard<std::mutex> lock(logMutex_);
  log_.clear();
}

MessageLog::Log MessageLog::getAndClearLog()
{
  const Log log = getLog();
  clearLog();
  return log;
}

}  // namespace common
}  // namespace soem_interface
