#ifndef DEFINITIONS_PID_H
#define DEFINITIONS_PID_H

#include "definitions_main.h"
#include "definitions_control.h"

enum pidConst{KP, KI, KD};


//! Scaling of PID Gains

const float SCALE_GAINS_ANGULAR_POSITION = 1e-4f * RAD_TO_DEG;
const float SCALE_GAINS_ANGULAR_SPEED = 1e-5f * RAD_TO_DEG;


//! Filters
const float POS_PID_FILTER_GAINS = 0.85f;
const float VEL_PID_FILTER_GAINS = 0.85f;

const float GT_KP_POSITION = 1.0f * SCALE_GAINS_ANGULAR_POSITION ; //[N/m]

const float GT_KI_POSITION = 0.0f * SCALE_GAINS_ANGULAR_POSITION ; //[N/m]

const float GT_KD_POSITION = 1.0f * SCALE_GAINS_ANGULAR_POSITION ; //[Ns/m]

const float GT_KP_SPEED =    1.0f * SCALE_GAINS_ANGULAR_SPEED ; //[N/m]

const float GT_KI_SPEED =    0.0f * SCALE_GAINS_ANGULAR_SPEED ; //[N/m]

const float GT_KD_SPEED =    1.0f * SCALE_GAINS_ANGULAR_SPEED ; //[Ns/m]

#endif //DEFINITIONS_PID_H
