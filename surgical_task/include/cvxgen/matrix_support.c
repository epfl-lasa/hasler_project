/* Produced by CVXGEN, 2020-04-23 09:30:45 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[7]*(params.J_1[0])-rhs[8]*(params.J_1[1])-rhs[9]*(params.J_1[2])-rhs[10]*(params.J_1[3])-rhs[11]*(params.J_1[4])-rhs[12]*(params.J_1[5])-rhs[13]*(params.J_1[6]);
  lhs[1] = -rhs[1]*(-1)-rhs[7]*(params.J_2[0])-rhs[8]*(params.J_2[1])-rhs[9]*(params.J_2[2])-rhs[10]*(params.J_2[3])-rhs[11]*(params.J_2[4])-rhs[12]*(params.J_2[5])-rhs[13]*(params.J_2[6]);
  lhs[2] = -rhs[2]*(-1)-rhs[7]*(params.J_3[0])-rhs[8]*(params.J_3[1])-rhs[9]*(params.J_3[2])-rhs[10]*(params.J_3[3])-rhs[11]*(params.J_3[4])-rhs[12]*(params.J_3[5])-rhs[13]*(params.J_3[6]);
  lhs[3] = -rhs[3]*(-1)-rhs[7]*(params.J_4[0])-rhs[8]*(params.J_4[1])-rhs[9]*(params.J_4[2])-rhs[10]*(params.J_4[3])-rhs[11]*(params.J_4[4])-rhs[12]*(params.J_4[5])-rhs[13]*(params.J_4[6]);
  lhs[4] = -rhs[4]*(-1)-rhs[7]*(params.J_5[0])-rhs[8]*(params.J_5[1])-rhs[9]*(params.J_5[2])-rhs[10]*(params.J_5[3])-rhs[11]*(params.J_5[4])-rhs[12]*(params.J_5[5])-rhs[13]*(params.J_5[6]);
  lhs[5] = -rhs[5]*(-1)-rhs[7]*(params.J_6[0])-rhs[8]*(params.J_6[1])-rhs[9]*(params.J_6[2])-rhs[10]*(params.J_6[3])-rhs[11]*(params.J_6[4])-rhs[12]*(params.J_6[5])-rhs[13]*(params.J_6[6]);
  lhs[6] = -rhs[6]*(-1)-rhs[7]*(params.J_7[0])-rhs[8]*(params.J_7[1])-rhs[9]*(params.J_7[2])-rhs[10]*(params.J_7[3])-rhs[11]*(params.J_7[4])-rhs[12]*(params.J_7[5])-rhs[13]*(params.J_7[6]);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[6]*(-1);
  lhs[7] = -rhs[0]*(params.J_1[0])-rhs[1]*(params.J_2[0])-rhs[2]*(params.J_3[0])-rhs[3]*(params.J_4[0])-rhs[4]*(params.J_5[0])-rhs[5]*(params.J_6[0])-rhs[6]*(params.J_7[0]);
  lhs[8] = -rhs[0]*(params.J_1[1])-rhs[1]*(params.J_2[1])-rhs[2]*(params.J_3[1])-rhs[3]*(params.J_4[1])-rhs[4]*(params.J_5[1])-rhs[5]*(params.J_6[1])-rhs[6]*(params.J_7[1]);
  lhs[9] = -rhs[0]*(params.J_1[2])-rhs[1]*(params.J_2[2])-rhs[2]*(params.J_3[2])-rhs[3]*(params.J_4[2])-rhs[4]*(params.J_5[2])-rhs[5]*(params.J_6[2])-rhs[6]*(params.J_7[2]);
  lhs[10] = -rhs[0]*(params.J_1[3])-rhs[1]*(params.J_2[3])-rhs[2]*(params.J_3[3])-rhs[3]*(params.J_4[3])-rhs[4]*(params.J_5[3])-rhs[5]*(params.J_6[3])-rhs[6]*(params.J_7[3]);
  lhs[11] = -rhs[0]*(params.J_1[4])-rhs[1]*(params.J_2[4])-rhs[2]*(params.J_3[4])-rhs[3]*(params.J_4[4])-rhs[4]*(params.J_5[4])-rhs[5]*(params.J_6[4])-rhs[6]*(params.J_7[4]);
  lhs[12] = -rhs[0]*(params.J_1[5])-rhs[1]*(params.J_2[5])-rhs[2]*(params.J_3[5])-rhs[3]*(params.J_4[5])-rhs[4]*(params.J_5[5])-rhs[5]*(params.J_6[5])-rhs[6]*(params.J_7[5]);
  lhs[13] = -rhs[0]*(params.J_1[6])-rhs[1]*(params.J_2[6])-rhs[2]*(params.J_3[6])-rhs[3]*(params.J_4[6])-rhs[4]*(params.J_5[6])-rhs[5]*(params.J_6[6])-rhs[6]*(params.J_7[6]);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[7]*(-params.dt[0]);
  lhs[1] = -rhs[8]*(-params.dt[0]);
  lhs[2] = -rhs[9]*(-params.dt[0]);
  lhs[3] = -rhs[10]*(-params.dt[0]);
  lhs[4] = -rhs[11]*(-params.dt[0]);
  lhs[5] = -rhs[12]*(-params.dt[0]);
  lhs[6] = -rhs[13]*(-params.dt[0]);
  lhs[7] = -rhs[7]*(params.dt[0]);
  lhs[8] = -rhs[8]*(params.dt[0]);
  lhs[9] = -rhs[9]*(params.dt[0]);
  lhs[10] = -rhs[10]*(params.dt[0]);
  lhs[11] = -rhs[11]*(params.dt[0]);
  lhs[12] = -rhs[12]*(params.dt[0]);
  lhs[13] = -rhs[13]*(params.dt[0]);
  lhs[14] = -rhs[7]*(-1);
  lhs[15] = -rhs[8]*(-1);
  lhs[16] = -rhs[9]*(-1);
  lhs[17] = -rhs[10]*(-1);
  lhs[18] = -rhs[11]*(-1);
  lhs[19] = -rhs[12]*(-1);
  lhs[20] = -rhs[13]*(-1);
  lhs[21] = -rhs[7]*(1);
  lhs[22] = -rhs[8]*(1);
  lhs[23] = -rhs[9]*(1);
  lhs[24] = -rhs[10]*(1);
  lhs[25] = -rhs[11]*(1);
  lhs[26] = -rhs[12]*(1);
  lhs[27] = -rhs[13]*(1);
  lhs[28] = -rhs[0]*(-1);
  lhs[29] = -rhs[1]*(-1);
  lhs[30] = -rhs[2]*(-1);
  lhs[31] = -rhs[3]*(-1);
  lhs[32] = -rhs[4]*(-1);
  lhs[33] = -rhs[5]*(-1);
  lhs[34] = -rhs[6]*(-1);
  lhs[35] = -rhs[0]*(1);
  lhs[36] = -rhs[1]*(1);
  lhs[37] = -rhs[2]*(1);
  lhs[38] = -rhs[3]*(1);
  lhs[39] = -rhs[4]*(1);
  lhs[40] = -rhs[5]*(1);
  lhs[41] = -rhs[6]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[28]*(-1)-rhs[35]*(1);
  lhs[1] = -rhs[29]*(-1)-rhs[36]*(1);
  lhs[2] = -rhs[30]*(-1)-rhs[37]*(1);
  lhs[3] = -rhs[31]*(-1)-rhs[38]*(1);
  lhs[4] = -rhs[32]*(-1)-rhs[39]*(1);
  lhs[5] = -rhs[33]*(-1)-rhs[40]*(1);
  lhs[6] = -rhs[34]*(-1)-rhs[41]*(1);
  lhs[7] = -rhs[0]*(-params.dt[0])-rhs[7]*(params.dt[0])-rhs[14]*(-1)-rhs[21]*(1);
  lhs[8] = -rhs[1]*(-params.dt[0])-rhs[8]*(params.dt[0])-rhs[15]*(-1)-rhs[22]*(1);
  lhs[9] = -rhs[2]*(-params.dt[0])-rhs[9]*(params.dt[0])-rhs[16]*(-1)-rhs[23]*(1);
  lhs[10] = -rhs[3]*(-params.dt[0])-rhs[10]*(params.dt[0])-rhs[17]*(-1)-rhs[24]*(1);
  lhs[11] = -rhs[4]*(-params.dt[0])-rhs[11]*(params.dt[0])-rhs[18]*(-1)-rhs[25]*(1);
  lhs[12] = -rhs[5]*(-params.dt[0])-rhs[12]*(params.dt[0])-rhs[19]*(-1)-rhs[26]*(1);
  lhs[13] = -rhs[6]*(-params.dt[0])-rhs[13]*(params.dt[0])-rhs[20]*(-1)-rhs[27]*(1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.slack[0]);
  lhs[1] = rhs[1]*(2*params.slack[1]);
  lhs[2] = rhs[2]*(2*params.slack[2]);
  lhs[3] = rhs[3]*(2*params.slack[3]);
  lhs[4] = rhs[4]*(2*params.slack[4]);
  lhs[5] = rhs[5]*(2*params.slack[5]);
  lhs[6] = rhs[6]*(2*params.slack[6]);
  lhs[7] = rhs[7]*(2*params.damping[0]);
  lhs[8] = rhs[8]*(2*params.damping[1]);
  lhs[9] = rhs[9]*(2*params.damping[2]);
  lhs[10] = rhs[10]*(2*params.damping[3]);
  lhs[11] = rhs[11]*(2*params.damping[4]);
  lhs[12] = rhs[12]*(2*params.damping[5]);
  lhs[13] = rhs[13]*(2*params.damping[6]);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = 0;
  work.q[10] = 0;
  work.q[11] = 0;
  work.q[12] = 0;
  work.q[13] = 0;
}
void fillh(void) {
  work.h[0] = -(params.qlow[0]-params.q[0]);
  work.h[1] = -(params.qlow[1]-params.q[1]);
  work.h[2] = -(params.qlow[2]-params.q[2]);
  work.h[3] = -(params.qlow[3]-params.q[3]);
  work.h[4] = -(params.qlow[4]-params.q[4]);
  work.h[5] = -(params.qlow[5]-params.q[5]);
  work.h[6] = -(params.qlow[6]-params.q[6]);
  work.h[7] = -(params.q[0]-params.qup[0]);
  work.h[8] = -(params.q[1]-params.qup[1]);
  work.h[9] = -(params.q[2]-params.qup[2]);
  work.h[10] = -(params.q[3]-params.qup[3]);
  work.h[11] = -(params.q[4]-params.qup[4]);
  work.h[12] = -(params.q[5]-params.qup[5]);
  work.h[13] = -(params.q[6]-params.qup[6]);
  work.h[14] = -params.dqlow[0];
  work.h[15] = -params.dqlow[1];
  work.h[16] = -params.dqlow[2];
  work.h[17] = -params.dqlow[3];
  work.h[18] = -params.dqlow[4];
  work.h[19] = -params.dqlow[5];
  work.h[20] = -params.dqlow[6];
  work.h[21] = params.dqup[0];
  work.h[22] = params.dqup[1];
  work.h[23] = params.dqup[2];
  work.h[24] = params.dqup[3];
  work.h[25] = params.dqup[4];
  work.h[26] = params.dqup[5];
  work.h[27] = params.dqup[6];
  work.h[28] = -params.slacklow[0];
  work.h[29] = -params.slacklow[1];
  work.h[30] = -params.slacklow[2];
  work.h[31] = -params.slacklow[3];
  work.h[32] = -params.slacklow[4];
  work.h[33] = -params.slacklow[5];
  work.h[34] = -params.slacklow[6];
  work.h[35] = params.slackup[0];
  work.h[36] = params.slackup[1];
  work.h[37] = params.slackup[2];
  work.h[38] = params.slackup[3];
  work.h[39] = params.slackup[4];
  work.h[40] = params.slackup[5];
  work.h[41] = params.slackup[6];
}
void fillb(void) {
  work.b[0] = params.dx[0];
  work.b[1] = params.dx[1];
  work.b[2] = params.dx[2];
  work.b[3] = params.dx[3];
  work.b[4] = params.dx[4];
  work.b[5] = params.dx[5];
  work.b[6] = params.dx[6];
}
void pre_ops(void) {
}
