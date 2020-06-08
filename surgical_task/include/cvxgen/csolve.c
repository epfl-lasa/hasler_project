/* Produced by CVXGEN, 2020-04-23 09:30:43 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"delta", "dq"};
  const int num_var_names = 2;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_1");
  if (xm == NULL) {
    /* Attempt to pull J_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.J_1 or params.J{1}.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("J_1 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_1;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_2");
  if (xm == NULL) {
    /* Attempt to pull J_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.J_2 or params.J{2}.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("J_2 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_2;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_3");
  if (xm == NULL) {
    /* Attempt to pull J_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.J_3 or params.J{3}.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("J_3 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_3;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_4");
  if (xm == NULL) {
    /* Attempt to pull J_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.J_4 or params.J{4}.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("J_4 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_4;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_5");
  if (xm == NULL) {
    /* Attempt to pull J_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.J_5 or params.J{5}.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("J_5 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_5;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_6");
  if (xm == NULL) {
    /* Attempt to pull J_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.J_6 or params.J{6}.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("J_6 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_6;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_7");
  if (xm == NULL) {
    /* Attempt to pull J_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.J_7 or params.J{7}.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("J_7 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_7;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "damping");
  if (xm == NULL) {
    printf("could not find params.damping.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("damping must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter damping must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter damping must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter damping must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.damping;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "dqlow");
  if (xm == NULL) {
    printf("could not find params.dqlow.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("dqlow must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter dqlow must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter dqlow must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter dqlow must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.dqlow;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "dqup");
  if (xm == NULL) {
    printf("could not find params.dqup.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("dqup must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter dqup must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter dqup must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter dqup must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.dqup;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "dt");
  if (xm == NULL) {
    printf("could not find params.dt.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("dt must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter dt must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter dt must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter dt must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.dt;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "dx");
  if (xm == NULL) {
    printf("could not find params.dx.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("dx must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter dx must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter dx must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter dx must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.dx;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "q");
  if (xm == NULL) {
    printf("could not find params.q.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter q must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter q must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter q must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.q;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qlow");
  if (xm == NULL) {
    printf("could not find params.qlow.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("qlow must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qlow must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qlow must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qlow must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qlow;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qup");
  if (xm == NULL) {
    printf("could not find params.qup.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("qup must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qup must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qup must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qup must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qup;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "slack");
  if (xm == NULL) {
    printf("could not find params.slack.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("slack must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter slack must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter slack must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter slack must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.slack;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "slacklow");
  if (xm == NULL) {
    printf("could not find params.slacklow.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("slacklow must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter slacklow must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter slacklow must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter slacklow must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.slacklow;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "slackup");
  if (xm == NULL) {
    printf("could not find params.slackup.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("slackup must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter slackup must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter slackup must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter slackup must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.slackup;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 18) {
    printf("Error: %d parameters are invalid.\n", 18 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 7; i++)
      printf("  params.damping[%d] = %.6g;\n", i, params.damping[i]);
    for (i = 0; i < 7; i++)
      printf("  params.slack[%d] = %.6g;\n", i, params.slack[i]);
    for (i = 0; i < 7; i++)
      printf("  params.J_1[%d] = %.6g;\n", i, params.J_1[i]);
    for (i = 0; i < 7; i++)
      printf("  params.dx[%d] = %.6g;\n", i, params.dx[i]);
    for (i = 0; i < 7; i++)
      printf("  params.J_2[%d] = %.6g;\n", i, params.J_2[i]);
    for (i = 0; i < 7; i++)
      printf("  params.J_3[%d] = %.6g;\n", i, params.J_3[i]);
    for (i = 0; i < 7; i++)
      printf("  params.J_4[%d] = %.6g;\n", i, params.J_4[i]);
    for (i = 0; i < 7; i++)
      printf("  params.J_5[%d] = %.6g;\n", i, params.J_5[i]);
    for (i = 0; i < 7; i++)
      printf("  params.J_6[%d] = %.6g;\n", i, params.J_6[i]);
    for (i = 0; i < 7; i++)
      printf("  params.J_7[%d] = %.6g;\n", i, params.J_7[i]);
    for (i = 0; i < 7; i++)
      printf("  params.qlow[%d] = %.6g;\n", i, params.qlow[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q[%d] = %.6g;\n", i, params.q[i]);
    for (i = 0; i < 1; i++)
      printf("  params.dt[%d] = %.6g;\n", i, params.dt[i]);
    for (i = 0; i < 7; i++)
      printf("  params.qup[%d] = %.6g;\n", i, params.qup[i]);
    for (i = 0; i < 7; i++)
      printf("  params.dqlow[%d] = %.6g;\n", i, params.dqlow[i]);
    for (i = 0; i < 7; i++)
      printf("  params.dqup[%d] = %.6g;\n", i, params.dqup[i]);
    for (i = 0; i < 7; i++)
      printf("  params.slacklow[%d] = %.6g;\n", i, params.slacklow[i]);
    for (i = 0; i < 7; i++)
      printf("  params.slackup[%d] = %.6g;\n", i, params.slackup[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "delta", xm);
  dest = mxGetPr(xm);
  src = vars.delta;
  for (i = 0; i < 7; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "dq", xm);
  dest = mxGetPr(xm);
  src = vars.dq;
  for (i = 0; i < 7; i++) {
    *dest++ = *src++;
  }
}
