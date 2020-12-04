/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Provides ros-independent action server implementation.
 */

#pragma once

#include <atomic>
#include <functional>
#include <string>

#include <message_logger/message_logger.hpp>

#include "any_action/ActionServerInterface.hpp"

namespace any_action
{
template <typename Result_, typename Feedback_>
class ActionServer : public ActionServerInterface
{
public:
  // Type definitions
  using Result = Result_;
  using Feedback = Feedback_;
  using FeedbackCallback = std::function<void(const Feedback&)>;

public:
  //! Default constructor. Generates an empty feedback callback method.
  ActionServer() : ActionServer([](const Feedback&) {})
  {
  }

  /**
   * Constructor for custom feedback callback methods.
   * @param feedbackCallback Feedback callback method.
   */
  explicit ActionServer(FeedbackCallback feedbackCallback)
    : feedbackMethod_(std::move(feedbackCallback)), isPreemptRequested_{ false }, status_{ ActionStatus::NONE }
  {
  }

  /**
   * Copy constructor implementation for ActionServer
   * @param other ActionServer to copy into this.
   */
  ActionServer(const ActionServer& other)
    : feedbackMethod_{ other.feedbackMethod_ }
    , isPreemptRequested_{ other.isPreemptRequested_.load() }
    , status_{ other.status_ }
    , log_{ other.log_ }
    , result_{ other.result_ }
  {
  }

  /**
   * Copy operator implementation for ActionServer.
   * @param other ActionServer to copy into this.
   * @return this
   */
  ActionServer& operator=(const ActionServer& other)
  {
    feedbackMethod_ = other.feedbackMethod_;
    isPreemptRequested_ = other.isPreemptRequested_.load();
    status_ = other.status_;
    log_ = other.log_;
    result_ = other.result_;
    return *this;
  }

  //! @copydoc ActionServerInterface::getStatus
  ActionStatus getStatus() const override
  {
    return status_;
  }

  //! @copydoc ActionServerInterface::requestPreempt
  void requestPreempt() override
  {
    isPreemptRequested_ = true;
  }

  //! @copydoc ActionServerInterface::isPreemptRequested
  bool isPreemptRequested() const override
  {
    return isPreemptRequested_;
  }

  //! @copydoc ActionServerInterface::acceptNewGoal
  void acceptNewGoal() override
  {
    isPreemptRequested_ = false;
    status_ = ActionStatus::NONE;
    log_ = std::string{};
    result_ = Result();
  }

  //! @copydoc ActionServerInterface::isResultAvailable
  bool isResultAvailable() const override
  {
    return status_ != ActionStatus::NONE;
  }

  /**
   * Sets action status to SUCCEEDED. Called at the end of an action execution.
   * @param result  Result of the action.
   * @param log     Additional log information describing the success.
   */
  void setSucceeded(const Result& result = Result(), const std::string& log = std::string(""))
  {
    status_ = ActionStatus::SUCCEEDED;
    log_ = log;
    result_ = result;
  }

  /**
   * Sets action status to PREEMPTED. Called at the end of an action execution.
   * @param result  Result of the action.
   * @param log     Additional log information describing the preemption.
   */
  void setPreempted(const Result& result = Result(), const std::string& log = std::string(""))
  {
    status_ = ActionStatus::PREEMPTED;
    log_ = log;
    result_ = result;
  }

  /**
   * Sets action status to ABORTED. Called at the end of an action execution.
   * @param result  Result of the action.
   * @param log     Additional log information describing the abortion.
   */
  void setAborted(const Result& result = Result(), const std::string& log = std::string(""))
  {
    status_ = ActionStatus::ABORTED;
    log_ = log;
    result_ = result;
  }

  /**
   * Send feedback over callback method. Can be called multiple times in one action execution.
   * @param feedback  Current feedback of the action.
   */
  void sendFeedback(const Feedback& feedback) const
  {
    feedbackMethod_(feedback);
  }

  /**
   * Returns action result. Checks whether status and therefore result was set.
   * @return Result of the action. Default constructed Result if not yet available.
   */
  Result getResult() const
  {
    if (!isResultAvailable())
    {
      MELO_ERROR("Result is not yet available!");
      return Result();
    }
    return result_;
  }

private:
  //! Callback method to provide feedback to the client.
  FeedbackCallback feedbackMethod_;
  //! Flag indicating if client requested action preemption.
  std::atomic_bool isPreemptRequested_;
  //! Current status of the action, used by client to determine action success.
  ActionStatus status_;
  //! Message to provide a more detailed log to the client.
  std::string log_;
  //! Result of the action.
  Result result_;
};

}  // namespace any_action
