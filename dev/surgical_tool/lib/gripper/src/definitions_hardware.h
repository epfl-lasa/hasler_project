#ifndef DEFINITIONS_HARDWARE_H
#define DEFINITIONS_HARDWARE_H

#include "definitions_main.h"

//! Transmissions

const float CABLE_TRANSMISSION = 1.0f; 
const float GEARBOX_TRANSMISSION = 66.0f; //166165 Planetary Gearhead GP 32 A ∅32 mm, 0.75–4.5 Nm
const float ROTOR_INERTIA_GRIPPER = 10.8f * (1.0f/1000.0f) * ((1.0f*1.0f) / (100.0f * 100.0f)); //! [kg.m2]

//!Resolution
const float GRIPPER_RESOLUTION =    (2 * M_PI / (4 * 1024)) / CABLE_TRANSMISSION;           

//! Motors Datasheet 118752 RE 25 Ø25 mm, balais en graphite, 20 W
const float NOMINAL_SPEED_MOTOR_GRIPPER    = 8330.0F;      //! [rpm]
const float NOMINAL_CURRENT_MOTOR_GRIPPER = 1.16F  ;  //! [A]   
const float NOMINAL_TORQUE_MOTOR_GRIPPER  = 26.3F  ; //! [mNm]
const float TORQUE_CONSTANT_MOTOR_GRIPPER  = 23.4F  ; //! [mNm/A]
const float SPEED_CONSTANT_MOTOR_GRIPPER   = 408.0F ;  //![rpm/V]    

const float NOMINAL_SPEED_GEARBOX_GRIPPER   = 8330.0f / GEARBOX_TRANSMISSION ;      //! 126.21 [rpm]
const float NOMINAL_TORQUE_GEARBOX_GRIPPER   = 26.3f * GEARBOX_TRANSMISSION ;      //! 1735.8 [mNm]

const float WS_RANGE_GRIPPER = 115.0F * DEG_TO_RAD;                 //! [m]

const int ENCODERSIGN_GRIPPER=1;

const int MOTORSIGN_GRIPPER=1;

const int ADC_SIGN_GRIPPER = 1;     

const float ENCODERSCALE_GRIPPER = 1.0f/360.0f;

#endif // DEFINITIONS_HARDWARE_H