/**
 * @authors     Remo Diethelm, Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Coordinate space calculations.
 */

#pragma once

#include "geometry_utils/DimensionType.hpp"
#include "geometry_utils/PoseStamped.hpp"

namespace geometry_utils
{
class CoordinateSpace
{
public:
  //! Struct to hold output of closest point to line call
  struct ClosestPointOnLineSegmentInfo
  {
    //! Point on the segment which is closest to reference point.
    Eigen::Vector3d point_ = Eigen::Vector3d::Zero();
    //! Minimum distance from reference point to closest point on the line.
    double distance_ = std::numeric_limits<double>::max();
    //! True if computed point is closer to start point, false if closer to end point of the line segment
    bool isCloserToStart_ = false;
  };

  //! Struct to hold output of closest point to line call
  struct ClosestPoseOnLineSegmentInfo
  {
    //! Frames can be different and pose can fail
    bool success_ = false;
    //! Point on the segment which is closest to reference point.
    PoseStamped pose_{};
    //! Minimum distance from reference point to closest point on the line.
    double distance_ = std::numeric_limits<double>::max();
    //! True if computed point is closer to start point, false if closer to end point of the line segment
    bool isCloserToStart_ = false;
  };

  /**
   * Get the difference between two points.
   * @param point1 First point.
   * @param point2 Second point.
   * @return Difference between the points.
   */
  template <TranslationDimensionType Dim>
  static Eigen::Vector3d getDifferenceBetweenPoints(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);

  /**
   * Get the distance between two points.
   * @param point1 First point.
   * @param point2 Second point.
   * @return Distance between the points.
   */
  template <TranslationDimensionType Dim>
  static double getDistanceBetweenPoints(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);

  /*!
   * Get the distance between two poses in the given dimension.
   * If the dimension is mixed (translational and rotational), the rotational part will be weighted.
   * @param[in] pose1 pose 1.
   * @param[in] pose2 pose 2.
   * @param[in] rotationalDistanceScaling Scale rotational distances, also used as weighting against translational
   * distances[m/rad].
   * (default 1m = 1rad).
   * @return distance.
   */
  template <enum DimensionType Dim>
  static double getDistanceBetweenPoses(const PoseStamped& pose1, const PoseStamped& pose2,
                                        double rotationalDistanceScaling = 1.0);

  /*!
   * Wrap an angle into (-pi,pi].
   * @param[in] angle input angle.
   * @return wrapped angle.
   */
  static double wrapAngle(double angle);

  /*!
   * Get the distance between two angles within [0..Pi].
   * @param[in] angle1 angle 1.
   * @param[in] angle2 angle 2.
   * @return distance.
   */
  static double getDistanceBetweenAngles(double angle1, double angle2);

  /**
   * Checks if a point lies on the line segment from startPoint to endPoint.
   * @param point reference point.
   * @param startPoint line segment start point.
   * @param endPoint line segment end point.
   * @param tolerance distance tolerance. Default 1mm. (margin around line, perpendicular distance along line, circle
   * around start and end)
   * @return true, if point lies on the line segment.
   */
  template <TranslationDimensionType Dim>
  static bool isPointOnLineSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& startPoint,
                                   const Eigen::Vector3d& endPoint, double tolerance = 1e-3);

  /*!
   * Compute closest point on a line segment with respect to a given point, in the given translational dimension.
   * @param referencePoint reference point.
   * @param startPoint line segment start point.
   * @param endPoint line segment end point.
   * @return Information about to closest point on the line.
   */
  template <TranslationDimensionType Dim>
  static ClosestPointOnLineSegmentInfo calculateClosestPointInfoOnLineSegment(const Eigen::Vector3d& referencePoint,
                                                                              const Eigen::Vector3d& startPoint,
                                                                              const Eigen::Vector3d& endPoint);

  template <TranslationDimensionType Dim>
  static inline double calculateDistanceToClosestPointOnLineSegment(const Eigen::Vector3d& referencePoint,
                                                                    const Eigen::Vector3d& startPoint,
                                                                    const Eigen::Vector3d& endPoint)
  {
    return calculateClosestPointInfoOnLineSegment<Dim>(referencePoint, startPoint, endPoint).distance_;
  }

  template <TranslationDimensionType Dim>
  static inline Eigen::Vector3d calculateClosestPointOnLineSegment(const Eigen::Vector3d& referencePoint,
                                                                   const Eigen::Vector3d& startPoint,
                                                                   const Eigen::Vector3d& endPoint)
  {
    return calculateClosestPointInfoOnLineSegment<Dim>(referencePoint, startPoint, endPoint).point_;
  }

  /*!
   * Compute closest pose on a line segment with respect to a given point, in the given translational dimension.
   * @param referencePoint reference point.
   * @param startPose line segment start pose.
   * @param endPose line segment end pose.
   * @return Information about to closest pose on the line. Orientation interpolated between start and end.
   */
  template <TranslationDimensionType Dim>
  static ClosestPoseOnLineSegmentInfo calculateClosestPoseInfoOnLineSegment(const Eigen::Vector3d& referencePoint,
                                                                            const PoseStamped& startPose,
                                                                            const PoseStamped& endPose);
};

}  // namespace geometry_utils

#include "geometry_utils/CoordinateSpace.tpp"