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


template<typename T = double>
class vibrator {

private:



  const T defaultVibMagnitude = -240.0; //![N / m/s]
  const T defaultVibDecayRate = 60; //! [1/s]
  const T defaultVibFrequency = 32.6; //! [Hz]
 
  static vibrator *me;

  LP_Filterd* _vibFilter;

  enum vibrator_Status {STANDBY, VIBRATING, FINISHED};

  enum vibrator_Types {DECAYING_SINE, SINE, CHIRP, SQUARE, SAWTOOTH};

  T _vibMagnitude, _vibDecayRate, _vibFrequency;

  
  T* _vibInput;
  T _vibInputInit; //!e.g. initial attack speed
  T* _vibOutput;

  vibrator_Status _myStatus;

  bool _flagTrigger, _flagReset;

  ros::Time _myStartTime;

  ros::Duration _myDuration;
  
  
  // METHODS
public:
  vibrator(T* input, T* output,T magnitude, T decayRate, T frequency, T filterGain);
  vibrator(T* input, T* output, T magnitude);
  bool run(ros::Time myCurrentTime);
  bool finished();

  void start();
  void changeParams(T magnitude, T decayRate, T frequency);
  void reset();
  ~vibrator();

private:
  void update(ros::Time myCurrentTime);

};
#endif // __vibrator_H__