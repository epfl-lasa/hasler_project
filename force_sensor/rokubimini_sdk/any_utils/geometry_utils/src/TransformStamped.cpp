/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Transform stamped
 */

#include "geometry_utils/TransformStamped.hpp"

#include <iomanip>
#include <iostream>

namespace geometry_utils
{
TransformStamped::TransformStamped(std::string frame, std::string childFrame, const Time& stamp,
                                   const Transform& transform)
  : frame_(std::move(frame)), childFrame_(std::move(childFrame)), stamp_(stamp), transform_(transform)
{
}

bool TransformStamped::isEqual(const TransformStamped& other) const
{
  return other.frame_ == frame_ && other.childFrame_ == childFrame_ &&
         other.transform_.getPosition() == transform_.getPosition() &&
         other.transform_.getRotation() == transform_.getRotation();
}

bool TransformStamped::hasEqualStamp(const TransformStamped& other) const
{
  return other.stamp_ == stamp_;
}

void TransformStamped::print(std::ostream& os) const
{
  os << std::fixed;
  os << "pose_stamped:"
     << "\n  header:"
     << "\n    frame: \"" << frame_ << "\""
     << "\n    stamp: " << std::setprecision(4) << stamp_.toSeconds() << "\n  child frame: \"" << childFrame_ << "\"";
  os << "\n  transform:"
     << "\n    translation: [" << std::setprecision(4) << transform_.getPosition() << "]"
     << "\n    rotation: [" << std::setprecision(4) << transform_.getRotation() << "]";
}

std::ostream& operator<<(std::ostream& stream, const TransformStamped& transform)
{
  transform.print(stream);
  return stream;
}

}  // namespace geometry_utils
