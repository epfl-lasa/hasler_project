/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Tests PoseStamped implementation
 */

#include <gtest/gtest.h>

#include <kindr/Core>
#include <kindr/common/gtest_eigen.hpp>

#include "geometry_utils/PoseStamped.hpp"

namespace geometry_utils
{
class TestPoseStamped : public ::testing::Test
{
public:
  TestPoseStamped()
    : pose_{ "map", Time(1.2), Position(1.0, 2.0, 3.0),
             RotationQuaternion(kindr::QuaternionD(-0.28054, 0.43171, -0.61472, 0.59753).toUnitQuaternion()) }
  {
  }
  PoseStamped pose_;
};

TEST_F(TestPoseStamped, interpolate)
{
  PoseStamped start{ "map", Time(1.2), Position{ 1.2, 3.6, 4.5 },
                     RotationQuaternion(kindr::QuaternionD(0.921, 0.001, 0.369, -0.123).toUnitQuaternion()) };
  PoseStamped end{ "map", Time(1.2), Position{ 3.5, -4.7, 1.1 },
                   RotationQuaternion(kindr::QuaternionD(-0.361, 0.003, 0.885, -0.295).toUnitQuaternion()) };

  PoseStamped interpolated = start;
  double t = 0.46;
  ASSERT_TRUE(interpolated.interpolate(start, end, t));
  KINDR_ASSERT_DOUBLE_MX_EQ(interpolated.position_.toImplementation(), Eigen::Vector3d(2.258, -0.218, 2.936), 0.01, "");
  ASSERT_TRUE(interpolated.orientation_.isNear(
      RotationQuaternion(kindr::QuaternionD(0.446, 0.0027, 0.8491, -0.283).toUnitQuaternion()), 1e-3));
}

}  // namespace geometry_utils
