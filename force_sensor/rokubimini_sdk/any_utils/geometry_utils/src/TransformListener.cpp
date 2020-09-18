/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Transform listener abstraction.
 */

#include "geometry_utils/TransformListener.hpp"
#include <message_logger/message_logger.hpp>

namespace geometry_utils
{
bool TransformListener::getPoseStamped(PoseStamped* poseStamped, const std::string& poseFrame,
                                       const std::string& referenceFrame, const Time& time, double timeout) const
{
  // Get transformation
  TransformStamped transformStamped;
  if (!getTransformation(&transformStamped, referenceFrame, poseFrame, time, timeout))
  {
    MELO_WARN("[TransformListener] Could not get transformation to pose '%s'.", poseFrame.c_str());
    return false;
  }

  // Get pose stamped by transforming identity
  PoseStamped poseInReferenceFrame;
  poseInReferenceFrame.frame_ = poseFrame;
  poseInReferenceFrame.stamp_ = time;
  if (!poseInReferenceFrame.applyTransformation(transformStamped))
  {
    MELO_WARN("[TransformListener] Could not apply transformation to pose '%s'.", poseFrame.c_str());
    return false;
  }

  *poseStamped = poseInReferenceFrame;
  return true;
}

}  // namespace geometry_utils
