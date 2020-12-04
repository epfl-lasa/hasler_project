/**
 * @authors     Remo Diethelm, Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Coordinate space calculations.
 */

#include "geometry_utils/CoordinateSpace.hpp"
#include "geometry_utils/TransformListener.hpp"

namespace geometry_utils
{
template <>
Eigen::Vector3d
CoordinateSpace::getDifferenceBetweenPoints<TranslationDimensionType::Txy>(const Eigen::Vector3d& point1,
                                                                           const Eigen::Vector3d& point2)
{
  Eigen::Vector3d difference = Eigen::Vector3d::Zero();
  difference.head<2>() = point2.head<2>() - point1.head<2>();
  return difference;
}

template <>
Eigen::Vector3d
CoordinateSpace::getDifferenceBetweenPoints<TranslationDimensionType::Txyz>(const Eigen::Vector3d& point1,
                                                                            const Eigen::Vector3d& point2)
{
  return point2 - point1;
}

template <>
double CoordinateSpace::getDistanceBetweenPoints<TranslationDimensionType::Txy>(const Eigen::Vector3d& point1,
                                                                                const Eigen::Vector3d& point2)
{
  return getDifferenceBetweenPoints<TranslationDimensionType::Txy>(point1, point2).norm();
}

template <>
double CoordinateSpace::getDistanceBetweenPoints<TranslationDimensionType::Txyz>(const Eigen::Vector3d& point1,
                                                                                 const Eigen::Vector3d& point2)
{
  return getDifferenceBetweenPoints<TranslationDimensionType::Txyz>(point1, point2).norm();
}

template <>
double CoordinateSpace::getDistanceBetweenPoses<DimensionType::Txy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                    double /*rotationalDistanceScaling*/)
{
  return getDistanceBetweenPoints<TranslationDimensionType::Txy>(pose1.position_.toImplementation(),
                                                                 pose2.position_.toImplementation());
}

template <>
double CoordinateSpace::getDistanceBetweenPoses<DimensionType::Txyz>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                     double /*rotationalDistanceScaling*/)
{
  return getDistanceBetweenPoints<TranslationDimensionType::Txyz>(pose1.position_.toImplementation(),
                                                                  pose2.position_.toImplementation());
}

template <>
double CoordinateSpace::getDistanceBetweenPoses<DimensionType::Ry>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                   double rotationalDistanceScaling)
{
  return rotationalDistanceScaling * getDistanceBetweenAngles(pose1.getYaw(), pose2.getYaw());
}

template <>
double CoordinateSpace::getDistanceBetweenPoses<DimensionType::Rrpy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                     double rotationalDistanceScaling)
{
  return rotationalDistanceScaling * std::abs(pose1.orientation_.getDisparityAngle(pose2.orientation_));
}

template <>
double CoordinateSpace::getDistanceBetweenPoses<DimensionType::TxyRy>(const PoseStamped& pose1,
                                                                      const PoseStamped& pose2,
                                                                      double rotationalDistanceScaling)
{
  return getDistanceBetweenPoses<DimensionType::Txy>(pose1, pose2) +
         getDistanceBetweenPoses<DimensionType::Ry>(pose1, pose2, rotationalDistanceScaling);
}

template <>
double CoordinateSpace::getDistanceBetweenPoses<DimensionType::TxyzRy>(const PoseStamped& pose1,
                                                                       const PoseStamped& pose2,
                                                                       double rotationalDistanceScaling)
{
  return getDistanceBetweenPoses<DimensionType::Txyz>(pose1, pose2) +
         getDistanceBetweenPoses<DimensionType::Ry>(pose1, pose2, rotationalDistanceScaling);
}

template <>
double CoordinateSpace::getDistanceBetweenPoses<DimensionType::TxyzRrpy>(const PoseStamped& pose1,
                                                                         const PoseStamped& pose2,
                                                                         double rotationalDistanceScaling)
{
  return getDistanceBetweenPoses<DimensionType::Txyz>(pose1, pose2) +
         getDistanceBetweenPoses<DimensionType::Rrpy>(pose1, pose2, rotationalDistanceScaling);
}

double CoordinateSpace::wrapAngle(double angle)
{
  return kindr::wrapPosNegPI(angle);
}

double CoordinateSpace::getDistanceBetweenAngles(double angle1, double angle2)
{
  const double distance1 = std::abs(wrapAngle(angle1 - angle2));
  const double distance2 = std::abs(wrapAngle(angle2 - angle1));
  return std::min(distance1, distance2);
}

}  // namespace geometry_utils
