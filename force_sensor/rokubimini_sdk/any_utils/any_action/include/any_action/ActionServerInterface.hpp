/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Provides ros-independent action server interface.
 */

#pragma once

namespace any_action
{
//! Status of the action
enum class ActionStatus
{
  ABORTED = -2,   /*!< -2 Action was aborted due to an internal error */
  PREEMPTED = -1, /*!< -1 Action was preempted from caller side */
  NONE = 0,       /*!< 0  Initial action status */
  SUCCEEDED = 1   /*!< 1   Action has succeeded */
};

class ActionServerInterface
{
public:
  //! Default constructor
  ActionServerInterface() = default;

  //! Default destructor
  virtual ~ActionServerInterface() = default;

  //! @return Current status of the action. NONE if not set.
  virtual ActionStatus getStatus() const = 0;

  //! Request preempting the action.
  virtual void requestPreempt() = 0;

  //! @return true, if preempt requested.
  virtual bool isPreemptRequested() const = 0;

  //! Reset internal states s.t. new goal can be accepted.
  virtual void acceptNewGoal() = 0;

  //! @return True, if result is available.
  virtual bool isResultAvailable() const = 0;
};

}  // namespace any_action
