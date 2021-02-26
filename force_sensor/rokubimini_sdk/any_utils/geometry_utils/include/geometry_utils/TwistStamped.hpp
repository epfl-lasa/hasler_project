/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Twist stamped
 */

#pragma once

#include "geometry_utils/TransformStamped.hpp"
#include "geometry_utils/typedefs.hpp"

namespace geometry_utils
{
class TwistStamped
{
public:
  /*!
   * Constructor.
   */
  TwistStamped() = default;

  /*!
   * Constructor.
   * @param frame           Coordinate frame the twist is expressed in
   * @param stamp           Timestamp of the pose
   * @param linearVelocity  Linear velocity
   * @param angularVelocity Angular velocity
   */
  TwistStamped(std::string frame, const Time& stamp, const LinearVelocity& linearVelocity,
               const LocalAngularVelocity& angularVelocity);

  /*!
   * Destructor.
   */
  virtual ~TwistStamped() = default;

  /**
   * Checks if two twists are equal
   * @param other Twist to compare *this to
   * @return true, if twists are equal
   */
  bool isEqual(const TwistStamped& other) const;

  /**
   * Checks if two twists are near
   * @param other Twist to compare *this to
   * @param tolerance tolerance
   * @return true, if twists are near
   */
  bool isNear(const TwistStamped& other, double tolerance) const;

  /**
   * Checks if two twists have the same time stamp
   * @param other Twist to compare *this to
   * @return true, if twists have the same time stamp
   */
  bool hasEqualStamp(const TwistStamped& other) const;

  /**
   * Apply a transformation to the pose type
   * @param transformStamped Transform
   * @return True, if twist can be transformed
   */
  bool applyTransformation(const TransformStamped& transformStamped);

  /**
   * Print method that can be inherited from
   * @param os stream to print to
   */
  virtual void print(std::ostream& os) const;

  //! Frame
  std::string frame_;

  //! Time
  Time stamp_;

  //! Linear velocity
  LinearVelocity linearVelocity_;

  //! Angular velocity
  LocalAngularVelocity angularVelocity_;
};

std::ostream& operator<<(std::ostream& stream, const TwistStamped& twist);

}  // namespace geometry_utils
