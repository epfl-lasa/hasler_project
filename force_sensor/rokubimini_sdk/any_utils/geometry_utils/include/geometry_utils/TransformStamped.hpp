/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Transform stamped.
 */

#pragma once

#include "geometry_utils/typedefs.hpp"

namespace geometry_utils
{
/**
 * Represents the transform from child frame to this frame.
 */
class TransformStamped
{
public:
  /*!
   * Constructor.
   */
  TransformStamped() = default;

  /*!
   * Constructor.
   * @param frame           Source coordinate frame
   * @param childFrame      Target coordinate frame
   * @param stamp           Timestamp of the pose
   * @param transform       Transform
   */
  TransformStamped(std::string frame, std::string childFrame, const Time& stamp, const Transform& transform);

  /*!
   * Destructor.
   */
  virtual ~TransformStamped() = default;

  /**
   * Checks if two transforms are equal
   * @param other Transform to compare *this to
   * @return true, if transforms are equal
   */
  bool isEqual(const TransformStamped& other) const;

  /**
   * Checks if two transforms have the same time stamp
   * @param other Transform to compare *this to
   * @return true, if transforms have the same time stamp
   */
  bool hasEqualStamp(const TransformStamped& other) const;

  /**
   * Print method that can be inherited from
   * @param os stream to print to
   */
  virtual void print(std::ostream& os) const;

  //! Frame
  std::string frame_;

  //! Child frame
  std::string childFrame_;

  //! Time
  Time stamp_;

  //! Transform from coordinate frame frame_ to coordinate frame childFrame_
  Transform transform_;
};

std::ostream& operator<<(std::ostream& stream, const TransformStamped& transform);

}  // namespace geometry_utils
