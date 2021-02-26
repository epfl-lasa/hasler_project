#pragma once

// std
#include <cassert>
#include <deque>
#include <iostream>
#include <mutex>

// message logger
#include <message_logger/message_logger.hpp>

namespace soem_interface
{
namespace common
{
class MessageLog
{
public:
  using Log = std::deque<std::pair<message_logger::log::levels::Level, std::string>>;

protected:
  static constexpr size_t MAX_LOG_SIZE = 20;

  static std::mutex logMutex_;
  static Log log_;

public:
  static void insertMessage(message_logger::log::levels::Level level, const std::string& message);
  static Log getLog();
  static void clearLog();
  static Log getAndClearLog();
};

}  // namespace common
}  // namespace soem_interface