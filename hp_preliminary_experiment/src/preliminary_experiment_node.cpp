#include "PreliminaryExperiment.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "preliminaryExperiment");
  ros::NodeHandle n;
  float frequency = 500.0f;

  PreliminaryExperiment experiment(n,frequency);

  if (!experiment.init()) 
  {
    return -1;
  }
  else
  {
    experiment.run();
  }
  return 0;
}
