# Stopwatch

Stopwatch class to profile execution times.

**Author(s):** Remo Diethelm

## Single measurement

Example code to measure the time of a single function call:

```
// stopwatch
#include <stopwatch/stopwatch.hpp>

using namespace stopwatch;

int main(int argc, char** argv)
{
  // Create a new stopwatch.
  Stopwatch sw("ExampleStopwatch");
  
  // Measure the time of a single function call.
  sw.start();
  sleep(1);
  sw.stop();
  
  // Check how long the function call took.
  const double measurement = sw.getStatistics().getLastMeasurement();
  
  return 0;
}
```

## Single measurement of two function calls

Example code to get the time of two function calls as a single measurment using the pause functionality:

```
// stopwatch
#include <stopwatch/stopwatch.hpp>

using namespace stopwatch;

int main(int argc, char** argv)
{
  // Create a new stopwatch.
  Stopwatch sw("ExampleStopwatch");
  
  // Include the time of the first function call by starting the stopwatch.
  sw.start();
  sleep(1);
  // Do not include the time of the second function call by pausing the stopwatch.
  sw.pause();
  sleep(2);
  // Include the time of the third function call by resuming the stopwatch.
  sw.resume();
  sleep(3);
  // Take the measurement by stopping the stopwatch.
  sw.stop();
  
  // Check how long the two function calls took (will be approx. 4 seconds).
  const double measurement = sw.getStatistics().getLastMeasurement();
  
  return 0;
}
```

## Multiple measurements

Example code to measure the time of multiple function calls:

```
// stopwatch
#include <stopwatch/stopwatch.hpp>

using namespace stopwatch;

int main(int argc, char** argv)
{
  // Create a new stopwatch.
  Stopwatch sw("ExampleStopwatch");
  
  // Measure the time of multiple function calls.
  for (unsigned int i = 0; i < 10; i++) {
    sw.start();
    sleep(1);
    sw.stop();
  }
  
  // Check how long the function calls took in average and their standard deviation.
  const double mean = sw.getStatistics().getMean();
  const double stdDev = sw.getStatistics().getStdDev();
  const double min = sw.getStatistics().getMin();
  const double max = sw.getStatistics().getMax();
  
  return 0;
}
```

## Access your stopwatch from anywhere

Example code to show how stopwatches can be accessed from anywhere using the "Stopwatches" class:

```
// stopwatch
#include <stopwatch/stopwatch.hpp>

using namespace stopwatch;

int main(int argc, char** argv)
{
  // Create a new stopwatch.
  StopwatchPtr sw = Stopwatches::getOrCreate("ExampleStopwatch");
  
  // Get the same stopwatch by name.
  StopwatchPtr sameSw = Stopwatches::getOrCreate("ExampleStopwatch");
  
  return 0;
}
```

