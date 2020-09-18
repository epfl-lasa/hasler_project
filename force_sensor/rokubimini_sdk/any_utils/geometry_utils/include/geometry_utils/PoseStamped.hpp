/**
 * @authors     Remo Diethelm, Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Pose stamped
 */

#pragma once

#include "geometry_utils/TransformStamped.hpp"
#include "geometry_utils/typedefs.hpp"

namespace geometry_utils
{
class PoseStamped
{
public:
  /*!
   * Constructor.
   */
  PoseStamped() = default;

  /*!
   * Constructor.
   * @param frame       Coordinate frame the pose is expressed in
   * @param stamp       Timestamp of the pose
   * @param position    Position
   * @param orientation Orientation
   */
  PoseStamped(std::string frame, const Time& stamp, const Position& position, const RotationQuaternion& orientation);

  /*!
   * Destructor.
   */
  virtual ~PoseStamped() = default;

  /*!
   * Get yaw of the rotation projected to the xy-plane.
   * @return yaw.
   */
  double getYaw() const;

  /**
   * Checks if two poses are equal
   * @param other Pose to compare *this to
   * @return true, if poses are equal
   */
  bool isEqual(const PoseStamped& other) const;

  /**
   * Checks if two poses are near
   * @param other Pose to compare *this to
   * @param positionTolerance tolerance in position
   * @param positionTolerance tolerance in orientation
   * @return true, if poses are near
   */
  bool isNear(const PoseStamped& other, double positionTolerance, double orientationTolerance) const;

  /**
   * Checks if two poses have the same time stamp
   * @param other Pose to compare *this to
   * @return true, if poses have the same time stamp
   */
  bool hasEqualStamp(const PoseStamped& other) const;

  /**
   * Apply a transformation to the pose type.
   * @param transformStamped Transform
   * @return True, if pose can be transformed
   */
  bool applyTransformation(const TransformStamped& transformStamped);

  /**
   * Interpolate current pose between pose1 and pose2.
   * @param pose1 Interpolation start pose
   * @param pose2 Interpolation end pose
   * @param t Interpolation factor (for interpolation: [0,1.0]
   * @param allowExtrapolation If extrapolation is allowed no range check on t is performed.
   * @return True, if pose can be interpolated
   */
  bool interpolate(const PoseStamped& pose1, const PoseStamped& pose2, double t, bool allowExtrapolation = false);

  /**
   * Print method that can be inherited from
   * @param os stream to print to
   */
  virtual void print(std::ostream& os) const;

  //! Frame
  std::string frame_;

  //! Time
  Time stamp_;

  //! Position
  Position position_;

  //! Orientation (is initialized with identity)
  RotationQuaternion orientation_;
};

std::ostream& operator<<(std::ostream& stream, const PoseStamped& pose);

}  // namespace geometry_utils
