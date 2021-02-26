// std
#include <thread>

// gtest
#include <gtest/gtest.h>

// stopwatch
#include "stopwatch/stopwatch.hpp"

using namespace stopwatch;

void thread_sleep(const double duration)
{
  std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1e9 * duration)));
}

TEST(Test, getOrCreate)
{
  StopwatchPtr sw1 = Stopwatches::getOrCreate("getOrCreate1");
  StopwatchPtr sw2 = Stopwatches::getOrCreate("getOrCreate1");
  StopwatchPtr sw3 = Stopwatches::getOrCreate("getOrCreate2");

  EXPECT_EQ(sw1.get(), sw2.get());
  EXPECT_NE(sw1.get(), sw3.get());

  EXPECT_EQ(sw1->getName(), std::string("getOrCreate1"));
  EXPECT_EQ(sw2->getName(), std::string("getOrCreate1"));
  EXPECT_EQ(sw3->getName(), std::string("getOrCreate2"));
}

TEST(Test, NoMeasurements)
{
  StopwatchPtr sw = Stopwatches::getOrCreate("NoMeasurements");

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), 0u);
  EXPECT_TRUE(std::isnan(st.getLastMeasurement()));
  EXPECT_TRUE(std::isnan(st.getMean()));
  EXPECT_TRUE(std::isnan(st.getVar()));
  EXPECT_TRUE(std::isnan(st.getStdDev()));
  EXPECT_TRUE(std::isnan(st.getMin()));
  EXPECT_TRUE(std::isnan(st.getMax()));
}

TEST(Test, IsRunning)
{
  StopwatchPtr sw = Stopwatches::getOrCreate("IsRunning");

  EXPECT_FALSE(sw->isRunning());

  sw->start();
  EXPECT_TRUE(sw->isRunning());
  sw->stop();

  sw->start();
  EXPECT_TRUE(sw->isRunning());
  sw->pause();
  EXPECT_FALSE(sw->isRunning());
  sw->resume();
  EXPECT_TRUE(sw->isRunning());
  sw->stop();

  EXPECT_FALSE(sw->isRunning());
}

TEST(Test, SingleMeasurement)
{
  const double duration = 0.1;

  StopwatchPtr sw = Stopwatches::getOrCreate("SingleMeasurement");

  sw->start();
  thread_sleep(duration);
  sw->stop();

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), 1u);
  EXPECT_NEAR(st.getLastMeasurement(), duration, 2.0e-3);
  EXPECT_NEAR(st.getMean(), duration, 2.0e-3);
  EXPECT_TRUE(std::isnan(st.getVar()));
  EXPECT_TRUE(std::isnan(st.getStdDev()));
  EXPECT_EQ(st.getMin(), st.getLastMeasurement());
  EXPECT_EQ(st.getMax(), st.getLastMeasurement());
}

TEST(Test, SingleMeasurementWithPause)
{
  const double duration = 0.1;

  StopwatchPtr sw = Stopwatches::getOrCreate("SingleMeasurementWithPause");

  sw->start();
  thread_sleep(duration);
  sw->pause();
  thread_sleep(duration);
  sw->resume();
  thread_sleep(duration);
  sw->stop();

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), 1u);
  EXPECT_NEAR(st.getLastMeasurement(), 2.0 * duration, 2.0e-3);
  EXPECT_NEAR(st.getMean(), 2.0 * duration, 2.0e-3);
  EXPECT_TRUE(std::isnan(st.getVar()));
  EXPECT_TRUE(std::isnan(st.getStdDev()));
  EXPECT_EQ(st.getMin(), st.getLastMeasurement());
  EXPECT_EQ(st.getMax(), st.getLastMeasurement());
}

TEST(Test, MultipleMeasurements1)
{
  const double duration = 0.1;

  StopwatchPtr sw = Stopwatches::getOrCreate("MultipleMeasurements1");

  sw->start();
  thread_sleep(1.0 * duration);
  sw->stop();

  sw->start();
  thread_sleep(2.0 * duration);
  sw->stop();

  sw->start();
  thread_sleep(3.0 * duration);
  sw->stop();

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), 3u);
  EXPECT_NEAR(st.getLastMeasurement(), 3.0 * duration, 1.0e-3);
  EXPECT_NEAR(st.getMean(), 2.0 * duration, 3.0e-3);
  EXPECT_NEAR(st.getVar(), std::pow(duration, 2), 5.0e-4);
  EXPECT_NEAR(st.getStdDev(), std::abs(duration), 2.0e-3);
  EXPECT_NEAR(st.getMin(), duration, 5.0e-3);
  EXPECT_NEAR(st.getMax(), 3.0 * duration, 1.0e-3);
}

TEST(Test, MultipleMeasurements2)
{
  StopwatchPtr sw = Stopwatches::getOrCreate("MultipleMeasurements2");

  // Test 9 sleeps with different durations.
  const std::vector<double> durations = { 0.04, 0.02, 0.04, 0.07, 0.05, 0.05, 0.04, 0.09, 0.05 };
  for (unsigned int i = 0; i < durations.size(); i++)
  {
    sw->start();
    thread_sleep(durations[i]);
    sw->stop();
  }

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), durations.size());
  EXPECT_NEAR(st.getLastMeasurement(), *durations.rbegin(), 1.0e-3);
  EXPECT_NEAR(st.getMean(), 0.05, 5.0e-4);
  EXPECT_NEAR(st.getVar(), 0.0004, 1.0e-5);
  EXPECT_NEAR(st.getStdDev(), 0.02, 2.0e-4);
  EXPECT_NEAR(st.getMin(), 0.02, 1.0e-3);
  EXPECT_NEAR(st.getMax(), 0.09, 1.0e-3);
}

TEST(Test, ClearMultipleMeasurements)
{
  StopwatchPtr sw = Stopwatches::getOrCreate("ClearMultipleMeasurements");

  sw->start();
  sw->stop();

  sw->start();
  sw->stop();

  sw->start();
  sw->stop();

  sw->clear();

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), 0u);
  EXPECT_TRUE(std::isnan(st.getLastMeasurement()));
  EXPECT_TRUE(std::isnan(st.getMean()));
  EXPECT_TRUE(std::isnan(st.getVar()));
  EXPECT_TRUE(std::isnan(st.getStdDev()));
  EXPECT_TRUE(std::isnan(st.getMin()));
  EXPECT_TRUE(std::isnan(st.getMax()));
}

TEST(Test, Overhead)
{
  StopwatchPtr sw = Stopwatches::getOrCreate("Overhead");

  sw->start();
  sw->stop();

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), 1u);
  EXPECT_LE(st.getLastMeasurement(), 1.0e-6);
  EXPECT_LE(st.getMean(), 1.0e-6);
  EXPECT_TRUE(std::isnan(st.getVar()));
  EXPECT_TRUE(std::isnan(st.getStdDev()));
  EXPECT_EQ(st.getMin(), st.getLastMeasurement());
  EXPECT_EQ(st.getMax(), st.getLastMeasurement());
}

TEST(Test, UnexpectedUsage)
{
  const double duration = 0.1;

  StopwatchPtr sw = Stopwatches::getOrCreate("UnexpectedUsage");

  EXPECT_FALSE(sw->isRunning());
  sw->start();
  thread_sleep(duration);
  EXPECT_TRUE(sw->isRunning());
  sw->start();
  thread_sleep(duration);
  EXPECT_TRUE(sw->isRunning());

  sw->stop();
  thread_sleep(duration);
  EXPECT_FALSE(sw->isRunning());
  sw->stop();
  EXPECT_FALSE(sw->isRunning());

  const Statistics& st = sw->getStatistics();
  EXPECT_EQ(st.getNumMeasurements(), 1u);
  EXPECT_NEAR(st.getLastMeasurement(), 2.0 * duration, 2.0e-3);
  EXPECT_NEAR(st.getMean(), 2.0 * duration, 2.0e-3);
  EXPECT_TRUE(std::isnan(st.getVar()));
  EXPECT_TRUE(std::isnan(st.getStdDev()));
  EXPECT_EQ(st.getMin(), st.getLastMeasurement());
  EXPECT_EQ(st.getMax(), st.getLastMeasurement());
}
