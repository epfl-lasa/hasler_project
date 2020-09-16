#ifndef DEFINITIONS_CONTROL_H
#define DEFINITIONS_CONTROL_H

#include "definitions_main.h"

// Type of control
enum Controller {
  POSITION_CTRL,
  SPEED_CTRL,
};

enum MotorDir {FW, BW};

//! Sampling times and loops
#define COMM_LOOP 1500 //! [us] -> 600Hz..
#define CTRL_LOOP 500 //! [us] -> 500us = 2KHz  /50
const float VELOCITY_PID_SAMPLE_P = 4 * CTRL_LOOP; //!  [us]
const float ACC_SAMPLE_P = 5 * CTRL_LOOP;
const float ANALOG_SAMPLING_TIME = 8 * CTRL_LOOP;
const float POSITION_PID_SAMPLE_P = 2 * CTRL_LOOP; //! [us]
const float invSpeedSampT =(1.0f / ((float)VELOCITY_PID_SAMPLE_P * 1e-6f));
const float invAccSampT = (1.0f / ((float)ACC_SAMPLE_P * 1e-6f));


#define HOMING_OFFSET 1.0F
#define SPEED_D_HOMING 1.0F
//! Macros for ADC

#define ADC 1
#define EFFORT_M ADC

//! Input and output resolution
#define MY_PWM_RESOLUTION 16  // Bits
#define MY_PWM_FREQUENCY 100000 // Hz
#define MY_ADC_RESOLUTION 12  // Bits



#endif // DEFINITIONS_CONTROL_H