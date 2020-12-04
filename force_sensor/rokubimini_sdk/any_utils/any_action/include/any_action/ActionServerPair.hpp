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
template <typename Result_, typename Converter_, typename Action1_, typename Action2_>
class ActionServerPair : public ActionServerInterface
{
public:
  using Result = Result_;
  using Converter = Converter_;
  using Action1 = Action1_;
  using Action1Feedback = typename Action1::Feedback;
  using Action2 = Action2_;
  using Action2Feedback = typename Action2::Feedback;
  using FeedbackPairCallback = std::function<void(const Action1Feedback& f1, const Action2Feedback& f2)>;

public:
  //! Default constructor not using feedback
  ActionServerPair() : ActionServerPair([](const Action1Feedback& f1, const Action2Feedback& f2) {})
  {
  }

  /**
   * Constructor that uses a custom feedback callback.
   * @param feedbackCallback Feedback callback for action pair.
   */
  ActionServerPair(FeedbackPairCallback feedbackCallback)
    : firstAction_([this, feedbackCallback](const Action1Feedback& f) {
      firstFeedback_ = f;
      feedbackCallback(firstFeedback_, secondFeedback_);
    })
    , secondAction_([this, feedbackCallback](const Action2Feedback& f) {
      secondFeedback_ = f;
      feedbackCallback(firstFeedback_, secondFeedback_);
    })
  {
  }

  //! @copydoc ActionServerInterface::getStatus
  ActionStatus getStatus() const override
  {
    return static_cast<ActionStatus>(
        std::min(static_cast<int>(firstAction_.getStatus()), static_cast<int>(secondAction_.getStatus())));
  }

  //! @copydoc ActionServerInterface::requestPreempt
  void requestPreempt() override
  {
    firstAction_.requestPreempt();
    secondAction_.requestPreempt();
  }

  //! @copydoc ActionServerInterface::isPreemptRequested
  bool isPreemptRequested() const override
  {
    return firstAction_.isPreemptRequested() || secondAction_.isPreemptRequested();
  }

  //! @copydoc ActionServerInterface::acceptNewGoal
  void acceptNewGoal() override
  {
    firstAction_.acceptNewGoal();
    secondAction_.acceptNewGoal();
  }

  //! @copydoc ActionServerInterface::isResultAvailable
  bool isResultAvailable() const
  {
    return firstAction_.isResultAvailable() && secondAction_.isResultAvailable();
  }

  /**
   * Returns action result. Checks whether status and therefore result was set.
   * @return Result of the action. Default constructed Result if not yet available.
   */
  Result getResult() const
  {
    if (!isResultAvailable())
    {
      MELO_ERROR("[ActionServer]: Result is not yet available!");
      return Result();
    }
    return Converter::convertResult(firstAction_.getResult(), secondAction_.getResult());
  }

protected:
  //! Feedback of first action
  Action1Feedback firstFeedback_;
  //! Feedback of second action
  Action2Feedback secondFeedback_;
  //! First action in pair
  Action1 firstAction_;
  //! Second action in pair
  Action2 secondAction_;
};

}  // namespace any_action
