/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Transform listener abstraction.
 */

#pragma once

#include <memory>
#include <string>

#include <message_logger/message_logger.hpp>

#include "geometry_utils/PoseStamped.hpp"
#include "geometry_utils/TransformStamped.hpp"

namespace geometry_utils
{
class TransformListener
{
public:
  //! Default Constructor
  TransformListener() = default;

  //! Default Destructor
  virtual ~TransformListener() = default;

  /**
   * Get current time.
   * @return Current time.
   */
  virtual Time getCurrentTime() const
  {
    return Time().setNow();
  }

  /**
   * Checks if transformation to transform from sourceFrame to targetFrame is available.
   * @param targetFrame       Target frame.
   * @param sourceFrame       Source frame.
   * @param time              Time at which transform shall be queried.
   * @return                  True, if transform is available.
   */
  virtual bool canTransform(const std::string& targetFrame, const std::string& sourceFrame, const Time& time) const = 0;

  /**
   * Get transformation to transform entity from sourceFrame(childFrame_ in Transformstamped) to targetFrame(frame_ in
   * Transformstamped).
   * @param transformStamped  Transform from sourceFrame to targetFrame.
   * @param targetFrame       Target frame (transform is expressed in this frame).
   * @param sourceFrame       Source frame
   * @param time              Time at which transform shall be queried
   * @param timeout           Wait for timeout seconds to get transform.
   * @return                  True, if valid transform was set.
   */
  virtual bool getTransformation(TransformStamped* transformStamped, const std::string& targetFrame,
                                 const std::string& sourceFrame, const Time& time, double timeout) const = 0;

  //! @copydoc TransformListener::getTransformation with timeout of 0.05s
  bool getTransformation(TransformStamped* transformStamped, const std::string& targetFrame,
                         const std::string& sourceFrame, const Time& time) const
  {
    return getTransformation(transformStamped, targetFrame, sourceFrame, time, 0.05);
  }

  //! @copydoc TransformListener::getTransformation at current time and with timeout of 0.05s
  bool getTransformation(TransformStamped* transformStamped, const std::string& targetFrame,
                         const std::string& sourceFrame) const
  {
    return getTransformation(transformStamped, targetFrame, sourceFrame, getCurrentTime(), 0.05);
  }

  /**
   * Get Pose of poseFrame in referenceFrame.
   * @param poseStamped     Pose of poseFrame in referenceFrame.
   * @param poseFrame       Pose frame.
   * @param referenceFrame  Reference frame.
   * @param time            Time at which to lookup pose.
   * @param timeout         Wait for timeout seconds to get transform.
   * @return                True, if valid pose was set.
   */
  bool getPoseStamped(PoseStamped* poseStamped, const std::string& poseFrame, const std::string& referenceFrame,
                      const Time& time, double timeout) const;

  //! @copydoc TransformListener::getPoseStamped with timeout of 0.05s
  bool getPoseStamped(PoseStamped* poseStamped, const std::string& poseFrame, const std::string& referenceFrame,
                      const Time& time) const
  {
    return getPoseStamped(poseStamped, poseFrame, referenceFrame, time, 0.05);
  }

  //! @copydoc TransformListener::getPoseStamped at current time and with timeout of 0.05s
  bool getPoseStamped(PoseStamped* poseStamped, const std::string& poseFrame, const std::string& referenceFrame) const
  {
    return getPoseStamped(poseStamped, poseFrame, referenceFrame, getCurrentTime(), 0.05);
  }

  /**
   * Transform stamped geometry_utils entity into target_frame at the time defined by the stamp of the stampedType.
   * @tparam StampedType_ Stamped type. (e.g. PoseStamped, TwistStamped)
   * @param stampedType   Must have public members frame_, stamp_ and a public member function
   * applyTransformation(TransformStamped)
   * @param target_frame  Frame the stamped entity should be expressed in.
   * @return True, if successfully transformed.
   */
  template <typename StampedType_>
  bool transform(StampedType_* stampedType, const std::string& target_frame) const
  {
    return transform(stampedType, target_frame, stampedType->stamp_);
  }

  /**
   * Transform stamped geometry_utils entity into target_frame at target_time.
   * @tparam StampedType_  Stamped type. (e.g. PoseStamped, TwistStamped)
   * @param stampedType    Must have public member frame_ and a public member function
   * applyTransformation(TransformStamped)
   * @param target_frame   Frame the stamped entity should be expressed in.
   * @param target_time    Time at which transform should take place.
   * @return True, if successfully transformed.
   */
  template <typename StampedType_>
  bool transform(StampedType_* stampedType, const std::string& target_frame, const Time& target_time) const
  {
    TransformStamped transformStamped;
    if (!getTransformation(&transformStamped, target_frame, stampedType->frame_, target_time))
    {
      MELO_ERROR("[TransformListener]: Could not get transform from source frame %s to target frame %s.",
                 stampedType->frame_.c_str(), target_frame.c_str());
      return false;
    }
    return stampedType->applyTransformation(transformStamped);
  }

  /**
   * Transform stamped geometry_utils entity into target_frame at target_time.
   * @tparam StampedType_  Stamped type. (e.g. PoseStamped, TwistStamped)
   * @param stampedType    Must have public member frame_ and a public member function
   * applyTransformation(TransformStamped)
   * @param target_frame   Frame the stamped entity should be expressed in.
   * @param target_time    Time at which transform should take place.
   * @param fixed_frame    Non-moving frame which should be used to perform the conversion.
   * @return True, if successfully transformed.
   */
  template <typename StampedType_>
  bool transform(StampedType_* stampedType, const std::string& target_frame, const Time& target_time,
                 const std::string& fixed_frame) const
  {
    StampedType_ stampedTypeCopy = *stampedType;
    TransformStamped transformStamped;
    if (!getTransformation(&transformStamped, fixed_frame, stampedType->frame_, stampedType->stamp_) ||
        !stampedTypeCopy->applyTransformation(transformStamped))
    {
      MELO_ERROR("[TransformListener]: Could get transform from source frame %s to target frame %s.",
                 stampedType->frame_.c_str(), fixed_frame.c_str());
      return false;
    }
    if (!getTransformation(&transformStamped, target_frame, fixed_frame, target_time) ||
        !stampedTypeCopy->applyTransformation(transformStamped))
    {
      MELO_ERROR("[TransformListener]: Could get transform from source frame %s to target frame %s.",
                 fixed_frame.c_str(), target_frame.c_str());
      return false;
    }
    *stampedType = stampedTypeCopy;
    return true;
  }
};

using TransformListenerPtr = std::shared_ptr<TransformListener>;

}  // namespace geometry_utils
