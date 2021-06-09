#ifndef __smoothSignals_H__
#define __smoothSignals_H__


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
#include "Utils_math.h"

#include "time.h"

using namespace std;
using namespace Eigen;

template<typename T = double>
class smoothSignals {

public:

  enum smoothSignals_Type {SMOOTH_RISE, SMOOTH_FALL, SMOOTH_RISE_FALL, SINUSOID};
private:
  
  static smoothSignals *me;
  
  enum smoothSignals_Status {STANDBY, CHANGING, FINISHED};
 
  T* _signalOutput;

  smoothSignals_Status _myStatus;
  smoothSignals_Type _myType;

  bool _flagTrigger, _flagReset;

  ros::Time _myStartTime;

  ros::Duration _myElapsedTime;

  T _myTimeFreq;
  T _timeFreqBias;
  
  
  // METHODS
public:
  smoothSignals(smoothSignals_Type type, T* output, T TimeFreq, T Bias);
  bool run(ros::Time myCurrentTime);
  
  bool finished();

  void start();
  void changeParams(smoothSignals_Type type, T TimeFreq);
  void reset();
  ~smoothSignals();

private:
  void update(ros::Time myCurrentTime);

};
#endif // __smoothSignals_H__