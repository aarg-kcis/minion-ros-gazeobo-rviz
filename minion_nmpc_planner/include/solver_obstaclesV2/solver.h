/* Produced by CVXGEN, 2018-03-28 05:44:13 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double g[3];
  double f_obs_0[3];
  double R[3];
  double f_obs_1[3];
  double f_obs_2[3];
  double f_obs_3[3];
  double f_obs_4[3];
  double f_obs_5[3];
  double f_obs_6[3];
  double f_obs_7[3];
  double f_obs_8[3];
  double f_obs_9[3];
  double f_obs_10[3];
  double f_obs_11[3];
  double f_obs_12[3];
  double f_obs_13[3];
  double f_obs_14[3];
  double f_obs_15[3];
  double f_obs_16[3];
  double f_obs_17[3];
  double f_obs_18[3];
  double f_obs_19[3];
  double f_obs_20[3];
  double f_obs_21[3];
  double f_obs_22[3];
  double f_obs_23[3];
  double f_obs_24[3];
  double f_obs_25[3];
  double f_obs_26[3];
  double f_obs_27[3];
  double f_obs_28[3];
  double f_obs_29[3];
  double f_obs_30[3];
  double f_obs_31[3];
  double f_obs_32[3];
  double f_obs_33[3];
  double f_obs_34[3];
  double f_obs_35[3];
  double xN[6];
  double Q[6];
  double A[9];
  double x_0[6];
  double B[6];
  double x_min[6];
  double x_max[6];
  double u_min[3];
  double u_max[3];
  double *f_obs[36];
  double *x[1];
} Params;
typedef struct Vars_t {
  double *u_0; /* 3 rows. */
  double *u_1; /* 3 rows. */
  double *u_2; /* 3 rows. */
  double *u_3; /* 3 rows. */
  double *u_4; /* 3 rows. */
  double *u_5; /* 3 rows. */
  double *u_6; /* 3 rows. */
  double *u_7; /* 3 rows. */
  double *u_8; /* 3 rows. */
  double *u_9; /* 3 rows. */
  double *u_10; /* 3 rows. */
  double *u_11; /* 3 rows. */
  double *u_12; /* 3 rows. */
  double *u_13; /* 3 rows. */
  double *u_14; /* 3 rows. */
  double *u_15; /* 3 rows. */
  double *u_16; /* 3 rows. */
  double *u_17; /* 3 rows. */
  double *u_18; /* 3 rows. */
  double *u_19; /* 3 rows. */
  double *u_20; /* 3 rows. */
  double *u_21; /* 3 rows. */
  double *u_22; /* 3 rows. */
  double *u_23; /* 3 rows. */
  double *u_24; /* 3 rows. */
  double *u_25; /* 3 rows. */
  double *u_26; /* 3 rows. */
  double *u_27; /* 3 rows. */
  double *u_28; /* 3 rows. */
  double *u_29; /* 3 rows. */
  double *u_30; /* 3 rows. */
  double *u_31; /* 3 rows. */
  double *u_32; /* 3 rows. */
  double *u_33; /* 3 rows. */
  double *u_34; /* 3 rows. */
  double *u_35; /* 3 rows. */
  double *x_36; /* 6 rows. */
  double *x_1; /* 6 rows. */
  double *x_2; /* 6 rows. */
  double *x_3; /* 6 rows. */
  double *x_4; /* 6 rows. */
  double *x_5; /* 6 rows. */
  double *x_6; /* 6 rows. */
  double *x_7; /* 6 rows. */
  double *x_8; /* 6 rows. */
  double *x_9; /* 6 rows. */
  double *x_10; /* 6 rows. */
  double *x_11; /* 6 rows. */
  double *x_12; /* 6 rows. */
  double *x_13; /* 6 rows. */
  double *x_14; /* 6 rows. */
  double *x_15; /* 6 rows. */
  double *x_16; /* 6 rows. */
  double *x_17; /* 6 rows. */
  double *x_18; /* 6 rows. */
  double *x_19; /* 6 rows. */
  double *x_20; /* 6 rows. */
  double *x_21; /* 6 rows. */
  double *x_22; /* 6 rows. */
  double *x_23; /* 6 rows. */
  double *x_24; /* 6 rows. */
  double *x_25; /* 6 rows. */
  double *x_26; /* 6 rows. */
  double *x_27; /* 6 rows. */
  double *x_28; /* 6 rows. */
  double *x_29; /* 6 rows. */
  double *x_30; /* 6 rows. */
  double *x_31; /* 6 rows. */
  double *x_32; /* 6 rows. */
  double *x_33; /* 6 rows. */
  double *x_34; /* 6 rows. */
  double *x_35; /* 6 rows. */
  double *u[36];
  double *x[37];
} Vars;
typedef struct Workspace_t {
  double h[648];
  double s_inv[648];
  double s_inv_z[648];
  double b[216];
  double q[324];
  double rhs[1836];
  double x[1836];
  double *s;
  double *z;
  double *y;
  double lhs_aff[1836];
  double lhs_cc[1836];
  double buffer[1836];
  double buffer2[1836];
  double KKT[3453];
  double L[2361];
  double d[1836];
  double v[1836];
  double d_inv[1836];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_977932222464[1];
  double quad_177068883968[1];
  double quad_68309282816[1];
  double quad_607940841472[1];
  double quad_453715046400[1];
  double quad_291936759808[1];
  double quad_791751004160[1];
  double quad_279864328192[1];
  double quad_947339124736[1];
  double quad_82542182400[1];
  double quad_481307693056[1];
  double quad_623054573568[1];
  double quad_891154956288[1];
  double quad_138075525120[1];
  double quad_825259503616[1];
  double quad_31115100160[1];
  double quad_148322045952[1];
  double quad_466261803008[1];
  double quad_479865458688[1];
  double quad_170794270720[1];
  double quad_122098642944[1];
  double quad_991023837184[1];
  double quad_593550049280[1];
  double quad_241083359232[1];
  double quad_280043077632[1];
  double quad_61778243584[1];
  double quad_955623231488[1];
  double quad_165325033472[1];
  double quad_307221454848[1];
  double quad_667226509312[1];
  double quad_844324741120[1];
  double quad_842449596416[1];
  double quad_94712745984[1];
  double quad_494086397952[1];
  double quad_605720883200[1];
  double quad_724377415680[1];
  double quad_236996321280[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
