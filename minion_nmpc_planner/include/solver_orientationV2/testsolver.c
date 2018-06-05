/* Produced by CVXGEN, 2018-03-14 06:47:50 -0400.  */
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
  params.g[0] = 0.20319161029830202;
  params.g[1] = 0.8325912904724193;
  params.g[2] = -0.8363810443482227;
  params.g[3] = 0.04331042079065206;
  params.f_obs_0[0] = 1.5717878173906188;
  params.f_obs_0[1] = 1.5851723557337523;
  params.f_obs_0[2] = -1.497658758144655;
  params.f_obs_0[3] = -1.171028487447253;
  params.R[0] = 1.0514672033008299;
  params.R[1] = 1.4408098436506365;
  params.R[2] = 1.0298762108785668;
  params.R[3] = 1.456833224394711;
  params.xN[0] = 0.596576190459043;
  params.xN[1] = -0.8860508694080989;
  params.xN[2] = 0.7050196079205251;
  params.xN[3] = 0.3634512696654033;
  params.xN[4] = -1.9040724704913385;
  params.xN[5] = 0.23541635196352795;
  params.xN[6] = -0.9629902123701384;
  params.xN[7] = -0.3395952119597214;
  params.xN[8] = -0.865899672914725;
  params.xN[9] = 0.7725516732519853;
  params.xN[10] = -0.23818512931704205;
  params.xN[11] = -1.372529046100147;
  params.xN[12] = 0.17859607212737894;
  params.xN[13] = 1.1212590580454682;
  params.xN[14] = -0.774545870495281;
  params.xN[15] = -1.1121684642712744;
  params.L_term[0] = 1.3879712575556487;
  params.L_term[1] = 1.9363836498604305;
  params.L_term[2] = 1.9759954224729337;
  params.L_term[3] = 1.6723836759128137;
  params.L_term[4] = 1.9028341085383982;
  params.L_term[5] = 1.8457508712931792;
  params.L_term[6] = 1.3779940413288891;
  params.L_term[7] = 1.0922170088717242;
  params.L_term[8] = 1.6534109025235362;
  params.L_term[9] = 1.557840762388451;
  params.L_term[10] = 1.361564763062578;
  params.L_term[11] = 1.2250545048398318;
  params.L_term[12] = 1.4065199163762485;
  params.L_term[13] = 1.4689402486991692;
  params.L_term[14] = 1.2692355782510614;
  params.L_term[15] = 1.2917927742254327;
  params.f_obs_1[0] = -0.16925440270808823;
  params.f_obs_1[1] = 1.442135651787706;
  params.f_obs_1[2] = 0.34501161787128565;
  params.f_obs_1[3] = -0.8660485502711608;
  params.f_obs_2[0] = -0.8880899735055947;
  params.f_obs_2[1] = -0.1815116979122129;
  params.f_obs_2[2] = -1.17835862158005;
  params.f_obs_2[3] = -1.1944851558277074;
  params.f_obs_3[0] = 0.05614023926976763;
  params.f_obs_3[1] = -1.6510825248767813;
  params.f_obs_3[2] = -0.06565787059365391;
  params.f_obs_3[3] = -0.5512951504486665;
  params.f_obs_4[0] = 0.8307464872626844;
  params.f_obs_4[1] = 0.9869848924080182;
  params.f_obs_4[2] = 0.7643716874230573;
  params.f_obs_4[3] = 0.7567216550196565;
  params.f_obs_5[0] = -0.5055995034042868;
  params.f_obs_5[1] = 0.6725392189410702;
  params.f_obs_5[2] = -0.6406053441727284;
  params.f_obs_5[3] = 0.29117547947550015;
  params.f_obs_6[0] = -0.6967713677405021;
  params.f_obs_6[1] = -0.21941980294587182;
  params.f_obs_6[2] = -1.753884276680243;
  params.f_obs_6[3] = -1.0292983112626475;
  params.f_obs_7[0] = 1.8864104246942706;
  params.f_obs_7[1] = -1.077663182579704;
  params.f_obs_7[2] = 0.7659100437893209;
  params.f_obs_7[3] = 0.6019074328549583;
  params.f_obs_8[0] = 0.8957565577499285;
  params.f_obs_8[1] = -0.09964555746227477;
  params.f_obs_8[2] = 0.38665509840745127;
  params.f_obs_8[3] = -1.7321223042686946;
  params.f_obs_9[0] = -1.7097514487110663;
  params.f_obs_9[1] = -1.2040958948116867;
  params.f_obs_9[2] = -1.3925560119658358;
  params.f_obs_9[3] = -1.5995826216742213;
  params.f_obs_10[0] = -1.4828245415645833;
  params.f_obs_10[1] = 0.21311092723061398;
  params.f_obs_10[2] = -1.248740700304487;
  params.f_obs_10[3] = 1.808404972124833;
  params.f_obs_11[0] = 0.7264471152297065;
  params.f_obs_11[1] = 0.16407869343908477;
  params.f_obs_11[2] = 0.8287224032315907;
  params.f_obs_11[3] = -0.9444533161899464;
  params.f_obs_12[0] = 1.7069027370149112;
  params.f_obs_12[1] = 1.3567722311998827;
  params.f_obs_12[2] = 0.9052779937121489;
  params.f_obs_12[3] = -0.07904017565835986;
  params.f_obs_13[0] = 1.3684127435065871;
  params.f_obs_13[1] = 0.979009293697437;
  params.f_obs_13[2] = 0.6413036255984501;
  params.f_obs_13[3] = 1.6559010680237511;
  params.f_obs_14[0] = 0.5346622551502991;
  params.f_obs_14[1] = -0.5362376605895625;
  params.f_obs_14[2] = 0.2113782926017822;
  params.f_obs_14[3] = -1.2144776931994525;
  params.f_obs_15[0] = -1.2317108144255875;
  params.f_obs_15[1] = 0.9026784957312834;
  params.f_obs_15[2] = 1.1397468137245244;
  params.f_obs_15[3] = 1.8883934547350631;
  params.theta_mates[0] = 1.4038856681660068;
  params.theta_mates[1] = 0.17437730638329096;
  params.Q[0] = -1.6408365219077408;
  params.Q[1] = -0.04450702153554875;
  params.A[0] = 1.7117453902485025;
  params.A[1] = 1.1504727980139053;
  params.A[2] = -0.05962309578364744;
  params.A[3] = -0.1788825540764547;
  params.A[4] = -1.1280569263625857;
  params.A[5] = -1.2911464767927057;
  params.A[6] = -1.7055053231225696;
  params.A[7] = 1.56957275034837;
  params.A[8] = 0.5607064675962357;
  params.A[9] = -1.4266707301147146;
  params.A[10] = -0.3434923211351708;
  params.A[11] = -1.8035643024085055;
  params.A[12] = -1.1625066019105454;
  params.A[13] = 0.9228324965161532;
  params.A[14] = 0.6044910817663975;
  params.A[15] = -0.0840868104920891;
  params.A[16] = -0.900877978017443;
  params.A[17] = 0.608892500264739;
  params.A[18] = 1.8257980452695217;
  params.A[19] = -0.25791777529922877;
  params.A[20] = -1.7194699796493191;
  params.A[21] = -1.7690740487081298;
  params.A[22] = -1.6685159248097703;
  params.A[23] = 1.8388287490128845;
  params.A[24] = 0.16304334474597537;
  params.A[25] = 1.3498497306788897;
  params.A[26] = -1.3198658230514613;
  params.A[27] = -0.9586197090843394;
  params.x_0[0] = 0.7679100474913709;
  params.x_0[1] = 1.5822813125679343;
  params.x_0[2] = -0.6372460621593619;
  params.x_0[3] = -1.741307208038867;
  params.x_0[4] = 1.456478677642575;
  params.x_0[5] = -0.8365102166820959;
  params.x_0[6] = 0.9643296255982503;
  params.x_0[7] = -1.367865381194024;
  params.x_0[8] = 0.7798537405635035;
  params.x_0[9] = 1.3656784761245926;
  params.x_0[10] = 0.9086083149868371;
  params.x_0[11] = -0.5635699005460344;
  params.x_0[12] = 0.9067590059607915;
  params.x_0[13] = -1.4421315032701587;
  params.x_0[14] = -0.7447235390671119;
  params.x_0[15] = -0.32166897326822186;
  params.B[0] = 1.5088481557772684;
  params.B[1] = -1.385039165715428;
  params.B[2] = 1.5204991609972622;
  params.B[3] = 1.1958572768832156;
  params.B[4] = 1.8864971883119228;
  params.B[5] = -0.5291880667861584;
  params.B[6] = -1.1802409243688836;
  params.x_min[0] = -1.037718718661604;
  params.x_min[1] = 1.3114512056856835;
  params.x_min[2] = 1.8609125943756615;
  params.x_min[3] = 0.7952399935216938;
  params.x_min[4] = -0.07001183290468038;
  params.x_min[5] = -0.8518009412754686;
  params.x_min[6] = 1.3347515373726386;
  params.x_min[7] = 1.4887180335977037;
  params.x_min[8] = -1.6314736327976336;
  params.x_min[9] = -1.1362021159208933;
  params.x_min[10] = 1.327044361831466;
  params.x_min[11] = 1.3932155883179842;
  params.x_min[12] = -0.7413880049440107;
  params.x_min[13] = -0.8828216126125747;
  params.x_min[14] = -0.27673991192616;
  params.x_min[15] = 0.15778600105866714;
  params.x_max[0] = -1.6177327399735457;
  params.x_max[1] = 1.3476485548544606;
  params.x_max[2] = 0.13893948140528378;
  params.x_max[3] = 1.0998712601636944;
  params.x_max[4] = -1.0766549376946926;
  params.x_max[5] = 1.8611734044254629;
  params.x_max[6] = 1.0041092292735172;
  params.x_max[7] = -0.6276245424321543;
  params.x_max[8] = 1.794110587839819;
  params.x_max[9] = 0.8020471158650913;
  params.x_max[10] = 1.362244341944948;
  params.x_max[11] = -1.8180107765765245;
  params.x_max[12] = -1.7774338357932473;
  params.x_max[13] = 0.9709490941985153;
  params.x_max[14] = -0.7812542682064318;
  params.x_max[15] = 0.0671374633729811;
  params.u_min[0] = -1.374950305314906;
  params.u_min[1] = 1.9118096386279388;
  params.u_min[2] = 0.011004190697677885;
  params.u_min[3] = 1.3160043138989015;
  params.u_max[0] = -1.7038488148800144;
  params.u_max[1] = -0.08433819112864738;
  params.u_max[2] = -1.7508820783768964;
  params.u_max[3] = 1.536965724350949;
}
