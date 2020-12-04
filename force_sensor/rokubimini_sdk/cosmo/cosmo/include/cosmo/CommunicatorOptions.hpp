/*!
 * @file	CommunicatorOptions.hpp
 * @author	Philipp Leemann
 * @date	Jul 24, 2017
 */

#pragma once

#include <boost/interprocess/permissions.hpp>

#include <string>
#include <chrono>

namespace cosmo
{
constexpr char DEFAULT_MEMORY_POOL_NAME[] = "COSMO_SHM";

class CommunicatorOptions
{
public:
  /**
   * @brief      Creates communicator options
   *
   * @param[in]  topic  The mesage topic
   */
  CommunicatorOptions(const std::string& topic)
    : topic_(topic)
    , memoryPoolName_(DEFAULT_MEMORY_POOL_NAME)
    , memoryPermissions_{ 0664 }
    ,  // set read+write for user and group, read-only for others
    missingProcessTimeout_{ 5000000 }
  {
  }

  virtual ~CommunicatorOptions() = default;

public:
  //! name of the topic to publish to / subscribe from
  std::string topic_;

  //! name of the shared memory block. Connections can not be made across blocks with different names.
  std::string memoryPoolName_;

  //! permissions for accessing the memory block. Unix style (e.g. 0664)
  boost::interprocess::permissions memoryPermissions_;

  //! Timeout [us] for the locking of the topic mutex during initialization. If this timeout is exceeded, it is assumed
  //! that a process has
  //! crashed while having the topic mutex locked.
  std::chrono::microseconds missingProcessTimeout_;
};

}  // namespace cosmo
