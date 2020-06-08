/* Produced by CVXGEN, 2020-04-23 09:30:45 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.damping[0] = 1.101595805149151;
  params.damping[1] = 1.4162956452362097;
  params.damping[2] = 0.5818094778258887;
  params.damping[3] = 1.021655210395326;
  params.damping[4] = 1.7858939086953094;
  params.damping[5] = 1.7925861778668761;
  params.damping[6] = 0.2511706209276725;
  params.slack[0] = 0.4144857562763735;
  params.slack[1] = 0.10293440660165976;
  params.slack[2] = 0.8816196873012729;
  params.slack[3] = 0.05975242175713391;
  params.slack[4] = 0.9136664487894222;
  params.slack[5] = 1.2982880952295215;
  params.slack[6] = 0.5569745652959506;
  params.J_1[0] = 0.7050196079205251;
  params.J_1[1] = 0.3634512696654033;
  params.J_1[2] = -1.9040724704913385;
  params.J_1[3] = 0.23541635196352795;
  params.J_1[4] = -0.9629902123701384;
  params.J_1[5] = -0.3395952119597214;
  params.J_1[6] = -0.865899672914725;
  params.dx[0] = 0.7725516732519853;
  params.dx[1] = -0.23818512931704205;
  params.dx[2] = -1.372529046100147;
  params.dx[3] = 0.17859607212737894;
  params.dx[4] = 1.1212590580454682;
  params.dx[5] = -0.774545870495281;
  params.dx[6] = -1.1121684642712744;
  params.J_2[0] = -0.44811496977740495;
  params.J_2[1] = 1.7455345994417217;
  params.J_2[2] = 1.9039816898917352;
  params.J_2[3] = 0.6895347036512547;
  params.J_2[4] = 1.6113364341535923;
  params.J_2[5] = 1.383003485172717;
  params.J_2[6] = -0.48802383468444344;
  params.J_3[0] = -1.631131964513103;
  params.J_3[1] = 0.6136436100941447;
  params.J_3[2] = 0.2313630495538037;
  params.J_3[3] = -0.5537409477496875;
  params.J_3[4] = -1.0997819806406723;
  params.J_3[5] = -0.3739203344950055;
  params.J_3[6] = -0.12423900520332376;
  params.J_4[0] = -0.923057686995755;
  params.J_4[1] = -0.8328289030982696;
  params.J_4[2] = -0.16925440270808823;
  params.J_4[3] = 1.442135651787706;
  params.J_4[4] = 0.34501161787128565;
  params.J_4[5] = -0.8660485502711608;
  params.J_4[6] = -0.8880899735055947;
  params.J_5[0] = -0.1815116979122129;
  params.J_5[1] = -1.17835862158005;
  params.J_5[2] = -1.1944851558277074;
  params.J_5[3] = 0.05614023926976763;
  params.J_5[4] = -1.6510825248767813;
  params.J_5[5] = -0.06565787059365391;
  params.J_5[6] = -0.5512951504486665;
  params.J_6[0] = 0.8307464872626844;
  params.J_6[1] = 0.9869848924080182;
  params.J_6[2] = 0.7643716874230573;
  params.J_6[3] = 0.7567216550196565;
  params.J_6[4] = -0.5055995034042868;
  params.J_6[5] = 0.6725392189410702;
  params.J_6[6] = -0.6406053441727284;
  params.J_7[0] = 0.29117547947550015;
  params.J_7[1] = -0.6967713677405021;
  params.J_7[2] = -0.21941980294587182;
  params.J_7[3] = -1.753884276680243;
  params.J_7[4] = -1.0292983112626475;
  params.J_7[5] = 1.8864104246942706;
  params.J_7[6] = -1.077663182579704;
  params.qlow[0] = 0.7659100437893209;
  params.qlow[1] = 0.6019074328549583;
  params.qlow[2] = 0.8957565577499285;
  params.qlow[3] = -0.09964555746227477;
  params.qlow[4] = 0.38665509840745127;
  params.qlow[5] = -1.7321223042686946;
  params.qlow[6] = -1.7097514487110663;
  params.q[0] = -1.2040958948116867;
  params.q[1] = -1.3925560119658358;
  params.q[2] = -1.5995826216742213;
  params.q[3] = -1.4828245415645833;
  params.q[4] = 0.21311092723061398;
  params.q[5] = -1.248740700304487;
  params.q[6] = 1.808404972124833;
  params.dt[0] = 0.7264471152297065;
  params.qup[0] = 0.16407869343908477;
  params.qup[1] = 0.8287224032315907;
  params.qup[2] = -0.9444533161899464;
  params.qup[3] = 1.7069027370149112;
  params.qup[4] = 1.3567722311998827;
  params.qup[5] = 0.9052779937121489;
  params.qup[6] = -0.07904017565835986;
  params.dqlow[0] = 1.3684127435065871;
  params.dqlow[1] = 0.979009293697437;
  params.dqlow[2] = 0.6413036255984501;
  params.dqlow[3] = 1.6559010680237511;
  params.dqlow[4] = 0.5346622551502991;
  params.dqlow[5] = -0.5362376605895625;
  params.dqlow[6] = 0.2113782926017822;
  params.dqup[0] = -1.2144776931994525;
  params.dqup[1] = -1.2317108144255875;
  params.dqup[2] = 0.9026784957312834;
  params.dqup[3] = 1.1397468137245244;
  params.dqup[4] = 1.8883934547350631;
  params.dqup[5] = 1.4038856681660068;
  params.dqup[6] = 0.17437730638329096;
  params.slacklow[0] = -1.6408365219077408;
  params.slacklow[1] = -0.04450702153554875;
  params.slacklow[2] = 1.7117453902485025;
  params.slacklow[3] = 1.1504727980139053;
  params.slacklow[4] = -0.05962309578364744;
  params.slacklow[5] = -0.1788825540764547;
  params.slacklow[6] = -1.1280569263625857;
  params.slackup[0] = -1.2911464767927057;
  params.slackup[1] = -1.7055053231225696;
  params.slackup[2] = 1.56957275034837;
  params.slackup[3] = 0.5607064675962357;
  params.slackup[4] = -1.4266707301147146;
  params.slackup[5] = -0.3434923211351708;
  params.slackup[6] = -1.8035643024085055;
}
