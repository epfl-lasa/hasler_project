#ifndef __vibrator_H__
#define __vibrator_H__


#include "LP_Filterd.h"
#include "Eigen/Eigen"
#include <signal.h>
#include <mutex>
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include "ros/ros.h"

#include "time.h"

using namespace std;
using namespace Eigen;

class vibrator {

private:
  
  static vibrator *me;

  LP_Filterd* _vibFilter;

  enum vibrator_Status {STANDBY, VIBRATING, FINISHED};

  enum vibrator_Types {DECAYING_SINE, SINE, CHIRP, SQUARE, SAWTOOTH};

  double _vibMagnitude, _vibDecayRate, _vibFrequency;

  
  double* _vibInput;
  double _vibInputInit; //!e.g. initial attack speed
  double* _vibOutput;

  vibrator_Status _myStatus;

  bool _flagTrigger, _flagReset;

  ros::Time _myStartTime;

  ros::Duration _myDuration;
  
  
  // METHODS
public:
  vibrator(double* input, double* output,double magnitude, double decayRate, double frequency, double filterGain);
  vibrator(double* input, double* output, double magnitude);
  bool run(ros::Time myCurrentTime);
  bool finished();

  void start();
  void changeParams(double magnitude, double decayRate, double frequency);
  void reset();
  ~vibrator();

private:
  void update(ros::Time myCurrentTime);

};
#endif // __vibrator_H__