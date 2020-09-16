#ifndef DEFINITIONS_MAIN_H
#define DEFINITIONS_MAIN_H

#include "math.h"


#define NB_GRIPPERS 2

#define NB_ACTIVE_GRIPPERS 1

//! Gripper type
#define GRIPPER_ID RIGHT_GRIPPER //! 1:Right 2:Left

//! Indexing
enum Lims{L_MIN, L_MAX, NB_LIMS};


//! Communication
//! For the serial communication look the main.cpp

//! Physics and Math Constants
#define GRAVITY -9.80665F
const float RAD_TO_DEG = 180.0f / M_PI;
const float DEG_TO_RAD = M_PI / 180.0f;

//! Features of gripper
extern const char *Platform_Names[];

#define GRIPPER_LIST                  \
  ListofGrippers(RIGHT_GRIPPER, "Right")\
      ListofGrippers(LEFT_GRIPPER, "Left")
#define ListofGrippers(enumeration, names) enumeration,
enum GrippersId : size_t { GRIPPER_LIST };
#undef ListofGrippers
extern const char *Grippers_Names[];

//! Operational Space
enum cartesianAxis { CART_X, CART_Y, CART_Z, NB_CART_AXIS };
#define NB_AXIS 5 //! Y, X, PITCH, ROLL, YAW

enum WrenchAxis { FX, FY, FZ, TX, TY, TZ, NB_AXIS_WRENCH };

//! State Machine
enum State {
  HOMING,
  EMERGENCY,
  STANDBY,
  RESET_UC,
  GRIPPER_STATE_CONTROL,
  NB_MACHINE_STATES
};

#endif // DEFINITIONS_MAIN_H