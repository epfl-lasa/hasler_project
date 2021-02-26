/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Main test file
 */

#include <gtest/gtest.h>
#include <ros/time.h>

using ::testing::InitGoogleTest;

/* RUN TESTS */
int main(int argc, char** argv)
{
  ros::Time::init();
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
