/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Twist stamped
 */

#include "geometry_utils/TwistStamped.hpp"

#include <iomanip>
#include <iostream>

namespace geometry_utils
{
TwistStamped::TwistStamped(std::string frame, const Time& stamp, const LinearVelocity& linearVelocity,
                           const LocalAngularVelocity& angularVelocity)  // NOLINT(modernize-pass-by-value)
    : frame_(std::move(frame)),
      stamp_(stamp),
      linearVelocity_(linearVelocity),
      angularVelocity_(angularVelocity)
{
}

bool TwistStamped::isEqual(const TwistStamped& other) const
{
  return other.frame_ == frame_ && other.linearVelocity_ == linearVelocity_ &&
         other.angularVelocity_ == angularVelocity_;
}

bool TwistStamped::isNear(const TwistStamped& other, double tolerance) const
{
  return other.frame_ == frame_ && ((other.linearVelocity_ - linearVelocity_).norm() <= tolerance) &&
         ((other.angularVelocity_ - angularVelocity_).norm() <= tolerance);
}

bool TwistStamped::hasEqualStamp(const TwistStamped& other) const
{
  return other.stamp_ == stamp_;
}

bool TwistStamped::applyTransformation(const TransformStamped& transformStamped)
{
  if (transformStamped.childFrame_ != frame_)
  {
    return false;
  }
  linearVelocity_ = transformStamped.transform_.getRotation().rotate(linearVelocity_);
  angularVelocity_ =
      transformStamped.transform_.getRotation().rotate(angularVelocity_);  // TODO(ghottiger) Add unit tests.
  frame_ = transformStamped.frame_;
  return true;
}

void TwistStamped::print(std::ostream& os) const
{
  os << std::fixed;
  os << "twist_stamped:"
     << "\n  header:"
     << "\n    frame: \"" << frame_ << "\""
     << "\n    stamp: " << std::setprecision(4) << stamp_.toSeconds();
  os << "\n  twist:"
     << "\n    linear velocity: [" << std::setprecision(4) << linearVelocity_ << "]"
     << "\n    angular velocity: [" << std::setprecision(4) << angularVelocity_ << "]";
}

std::ostream& operator<<(std::ostream& stream, const TwistStamped& twist)
{
  twist.print(stream);
  return stream;
}

}  // namespace geometry_utils
