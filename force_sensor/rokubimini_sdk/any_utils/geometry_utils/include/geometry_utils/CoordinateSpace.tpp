/**
 * @authors     Marco Tranzatto, Gabriel Hottiger
 * @affiliation RSL, ANYbotics
 * @brief       Coordinate space calculations.
 */

#pragma once

#include "geometry_utils/CoordinateSpace.hpp"
#include "geometry_utils/helper_methods.hpp"

namespace geometry_utils {

template <TranslationDimensionType Dim>
bool CoordinateSpace::isPointOnLineSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& startPoint, const Eigen::Vector3d& endPoint,
                                           double tolerance) {
  Eigen::Vector3d r1 = getDifferenceBetweenPoints<Dim>(startPoint, point);
  Eigen::Vector3d r2 = getDifferenceBetweenPoints<Dim>(startPoint, endPoint);

  // point lies on line
  if (r1.cross(r2.normalized()).norm() > tolerance) {
    return false;
  }

  // not behind start (tolerance circle around start point)
  double dotProduct = r1.dot(r2);
  if (dotProduct < 0) {
    return r1.norm() < tolerance;
  }

  // not behind end (tolerance circle around end point)
  const double r2Norm = r2.norm();
  if (dotProduct > (r2Norm * r2Norm)) {
    return getDistanceBetweenPoints<Dim>(endPoint, point) < tolerance;
  }

  return true;
}

template <TranslationDimensionType Dim>
CoordinateSpace::ClosestPointOnLineSegmentInfo CoordinateSpace::calculateClosestPointInfoOnLineSegment(
    const Eigen::Vector3d& referencePoint, const Eigen::Vector3d& startPoint, const Eigen::Vector3d& endPoint) {
  ClosestPointOnLineSegmentInfo closestPointInfo;
  // Calculate distance to startPoint if the line is negligible
  const double lineLength = CoordinateSpace::getDistanceBetweenPoints<Dim>(startPoint, endPoint);
  if (almostEqual(lineLength, 0.0)) {
    closestPointInfo.point_ = startPoint;
    closestPointInfo.distance_ = CoordinateSpace::getDistanceBetweenPoints<Dim>(startPoint, referencePoint);
    closestPointInfo.isCloserToStart_ = true;
  } else {
    // use only 3D position to compute closest point.
    Vector3D v_0_1{getDifferenceBetweenPoints<Dim>(startPoint, endPoint)};
    Vector3D v_ref_1{getDifferenceBetweenPoints<Dim>(referencePoint, endPoint)};
    double normalizedProjectionOnLine = (v_0_1.dot(v_ref_1)) / (v_0_1.dot(v_0_1));
    if (normalizedProjectionOnLine >= 1.0) {
      closestPointInfo.point_ = startPoint;
      closestPointInfo.isCloserToStart_ = true;
    } else if (normalizedProjectionOnLine <= 0.0) {
      closestPointInfo.point_ = endPoint;
      closestPointInfo.isCloserToStart_ = false;
    } else {
      // convex combination but do not consider orientation.
      closestPointInfo.point_ = normalizedProjectionOnLine * startPoint + (1.0 - normalizedProjectionOnLine) * endPoint;
      closestPointInfo.isCloserToStart_ = (normalizedProjectionOnLine >= 0.5);
    }
    closestPointInfo.distance_ = CoordinateSpace::getDistanceBetweenPoints<Dim>(referencePoint, closestPointInfo.point_);
  }
  return closestPointInfo;
}

template <TranslationDimensionType Dim>
CoordinateSpace::ClosestPoseOnLineSegmentInfo CoordinateSpace::calculateClosestPoseInfoOnLineSegment(const Eigen::Vector3d& referencePoint,
                                                                                                     const PoseStamped& startPose,
                                                                                                     const PoseStamped& endPose) {
  ClosestPoseOnLineSegmentInfo closestPoseInfo;
  closestPoseInfo.pose_ = startPose;

  // Get closest point on line
  auto closestPointInfo = calculateClosestPointInfoOnLineSegment<Dim>(referencePoint, startPose.position_.toImplementation(),
                                                                      endPose.position_.toImplementation());
  closestPoseInfo.distance_ = closestPointInfo.distance_;
  closestPoseInfo.isCloserToStart_ = closestPointInfo.isCloserToStart_;

  // Calculate interpolation factor and interpolate
  double distanceFromStart = getDistanceBetweenPoints<Dim>(startPose.position_.toImplementation(), closestPointInfo.point_);
  double segmentLength = getDistanceBetweenPoints<Dim>(startPose.position_.toImplementation(), endPose.position_.toImplementation());
  double t = std::max(0.0, std::min(1.0, distanceFromStart / segmentLength));
  closestPoseInfo.success_ = closestPoseInfo.pose_.interpolate(startPose, endPose, t);

  return closestPoseInfo;
}

}  // namespace geometry_utils
