/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Tests CoordinateSpace
 */

#include <gtest/gtest.h>

#include <kindr/common/gtest_eigen.hpp>

#include <geometry_utils/geometry_utils.hpp>

namespace geometry_utils
{
class TestPointOnLineSegment : public ::testing::Test
{
public:
  Eigen::Vector3d startPoint_{ 1.0, 5.0, 2.0 };
  Eigen::Vector3d endPoint_{ 4.0, 9.0, 5.0 };
};

class TestClosestPointOnLineSegment : public ::testing::Test
{
public:
  Eigen::Vector3d startPoint_{ 1.0, 5.0, 2.0 };
  Eigen::Vector3d endPoint_{ 4.0, 9.0, 5.0 };
};

class TestClosestPoseOnLineSegment : public ::testing::Test
{
public:
  PoseStamped startPose_{ "map", Time(1.2), Position{ 1.2, 3.6, 4.5 },
                          RotationQuaternion(kindr::QuaternionD(0.921, 0.001, 0.369, -0.123).toUnitQuaternion()) };
  PoseStamped endPose_{ "map", Time(1.2), Position{ 3.5, -4.7, 1.1 },
                        RotationQuaternion(kindr::QuaternionD(-0.361, 0.003, 0.885, -0.295).toUnitQuaternion()) };
};

TEST_F(TestPointOnLineSegment, beforeStartWithinTolerance)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 0.994855, 4.99314, 1.99486 };
  EXPECT_TRUE(
      CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.011));
}

TEST_F(TestPointOnLineSegment, beforeStartOutsideTolerance)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 0.994855, 4.99314, 1.99486 };
  EXPECT_FALSE(
      CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.009));
}

TEST_F(TestPointOnLineSegment, midPointWithinTolerance)
{                                         // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 2.49, 7, 3.5 };  // Distance to closest point on line 0.00857493
  EXPECT_TRUE(
      CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.009));
}

TEST_F(TestPointOnLineSegment, midPointOutsideTolerance)
{                                         // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 2.49, 7, 3.5 };  // Distance to closest point on line 0.00857493
  EXPECT_FALSE(
      CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.008));
}

TEST_F(TestPointOnLineSegment, afterEndwWithinTolerance)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 4.00514, 9.00686, 5.00514 };
  EXPECT_TRUE(
      CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.011));
}

TEST_F(TestPointOnLineSegment, afterEndOutsideTolerance)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 4.00514, 9.00686, 5.00514 };
  EXPECT_FALSE(
      CoordinateSpace::isPointOnLineSegment<TranslationDimensionType::Txyz>(point, startPoint_, endPoint_, 0.009));
}

TEST_F(TestClosestPointOnLineSegment, stopsAtStart)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 0.5, 4.5, 1.5 };

  auto closestPointOnLine = CoordinateSpace::calculateClosestPointInfoOnLineSegment<TranslationDimensionType::Txyz>(
      point, startPoint_, endPoint_);
  EXPECT_NEAR(closestPointOnLine.distance_, 0.866025403784439, 1e-10);
  EXPECT_EQ(closestPointOnLine.isCloserToStart_, true);
  EXPECT_TRUE(closestPointOnLine.point_.isApprox(startPoint_, 1e-10));
}

TEST_F(TestClosestPointOnLineSegment, stopsAtEnd)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 4.5, 9.5, 5.5 };
  auto closestPointOnLine = CoordinateSpace::calculateClosestPointInfoOnLineSegment<TranslationDimensionType::Txyz>(
      point, startPoint_, endPoint_);
  EXPECT_NEAR(closestPointOnLine.distance_, 0.866025403784439, 1e-10);
  EXPECT_EQ(closestPointOnLine.isCloserToStart_, false);
  EXPECT_TRUE(closestPointOnLine.point_.isApprox(endPoint_, 1e-10));
}

TEST_F(TestClosestPointOnLineSegment, findsClosestPointOnLine)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point{ 2.5, 6.5, 1.5 };
  double distance = CoordinateSpace::calculateDistanceToClosestPointOnLineSegment<TranslationDimensionType::Txyz>(
      point, startPoint_, endPoint_);
  EXPECT_NEAR(distance, 1.53871604229745, 1e-10);
}

TEST_F(TestClosestPoseOnLineSegment, findsClosestPoseOnLine)
{  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  Eigen::Vector3d point = Eigen::Vector3d{ 0, -3.4, 8.3 } + Eigen::Vector3d(2.258, -0.218, 2.936);
  auto info = CoordinateSpace::calculateClosestPoseInfoOnLineSegment<TranslationDimensionType::Txyz>(point, startPose_,
                                                                                                     endPose_);
  EXPECT_TRUE(info.success_);
  EXPECT_NEAR(info.distance_, 8.96939, 1e-5);
  EXPECT_TRUE(info.isCloserToStart_);
  KINDR_ASSERT_DOUBLE_MX_EQ(info.pose_.position_.toImplementation(), Eigen::Vector3d(2.258, -0.218, 2.936), 0.01, "");
  ASSERT_TRUE(info.pose_.orientation_.isNear(
      RotationQuaternion(kindr::QuaternionD(0.446, 0.0027, 0.8491, -0.283).toUnitQuaternion()), 1e-3));
}

}  // namespace geometry_utils
