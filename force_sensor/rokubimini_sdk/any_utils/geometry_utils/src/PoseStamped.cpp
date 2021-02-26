/**
 * @authors     Remo Diethelm, Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Pose stamped
 */

#include "geometry_utils/PoseStamped.hpp"

#include <iomanip>
#include <iostream>

#include <message_logger/message_logger.hpp>

namespace geometry_utils
{
// NOLINTNEXTLINE(modernize-pass-by-value)
PoseStamped::PoseStamped(std::string frame, const Time& stamp, const Position& position,
                         const RotationQuaternion& orientation)
  : frame_(std::move(frame)), stamp_(stamp), position_(position), orientation_(orientation)
{
}

double PoseStamped::getYaw() const
{
  return EulerAnglesZyx(orientation_).getUnique().z();
}

bool PoseStamped::isEqual(const PoseStamped& other) const
{
  return other.frame_ == frame_ && other.position_ == position_ && other.orientation_ == orientation_;
}

bool PoseStamped::isNear(const PoseStamped& other, double positionTolerance, double orientationTolerance) const
{
  return other.frame_ == frame_ && ((other.position_ - position_).norm() <= positionTolerance) &&
         other.orientation_.isNear(orientation_, orientationTolerance);
}

bool PoseStamped::hasEqualStamp(const PoseStamped& other) const
{
  return other.stamp_ == stamp_;
}

bool PoseStamped::applyTransformation(const TransformStamped& transformStamped)
{
  if (transformStamped.childFrame_ != frame_)
  {
    return false;
  }
  position_ = transformStamped.transform_.transform(position_);
  orientation_ = transformStamped.transform_.getRotation() * orientation_;  // TODO(ghottiger) Add unit tests.
  frame_ = transformStamped.frame_;
  return true;
}

bool PoseStamped::interpolate(const PoseStamped& pose1, const PoseStamped& pose2, const double t,
                              const bool allowExtrapolation)
{
  if (pose1.frame_ != pose2.frame_)
  {
    MELO_ERROR("Frame mismatch during interpolation.")
    return false;
  }
  if (!allowExtrapolation && (t < 0.0 || t > 1.0))
  {
    MELO_ERROR("Interpolation factor t is not in range [0, 1.0], but extrapolation is not allowed.")
    return false;
  }

  frame_ = pose1.frame_;
  position_ = pose1.position_ + (pose2.position_ - pose1.position_) * t;
  orientation_ = pose1.orientation_.boxPlus(t * (pose2.orientation_.boxMinus(pose1.orientation_)));
  return true;
}

void PoseStamped::print(std::ostream& os) const
{
  os << std::fixed;
  os << "pose_stamped:"
     << "\n  header:"
     << "\n    frame: \"" << frame_ << "\""
     << "\n    stamp: " << std::setprecision(4) << stamp_.toSeconds();
  os << "\n  pose:"
     << "\n    position: [" << std::setprecision(4) << position_ << "]"
     << "\n    orientation: [" << std::setprecision(4) << orientation_ << "]";
}

std::ostream& operator<<(std::ostream& stream, const PoseStamped& pose)
{
  pose.print(stream);
  return stream;
}

}  // namespace geometry_utils
