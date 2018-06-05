/* Produced by CVXGEN, 2016-12-22 06:28:40 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
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
  double R[3];
  double x_0[15];
  double xr_0[15];
  double L_stage[15];
  double xr_1[15];
  double xr_2[15];
  double xr_3[15];
  double xr_4[15];
  double xr_5[15];
  double xr_6[15];
  double xr_7[15];
  double xr_8[15];
  double xr_9[15];
  double xr_10[15];
  double xr_11[15];
  double xr_12[15];
  double xr_13[15];
  double xr_14[15];
  double xr_15[15];
  double xN[15];
  double L_term[15];
  double A[27];
  double B[6];
  double x_min[15];
  double x_max[15];
  double u_min[3];
  double u_max[3];
  double *x[1];
  double *xr[16];
} Params;
typedef struct Vars_t {
  double *u_0; /* 3 rows. */
  double *u_1; /* 3 rows. */
  double *x_1; /* 15 rows. */
  double *u_2; /* 3 rows. */
  double *x_2; /* 15 rows. */
  double *u_3; /* 3 rows. */
  double *x_3; /* 15 rows. */
  double *u_4; /* 3 rows. */
  double *x_4; /* 15 rows. */
  double *u_5; /* 3 rows. */
  double *x_5; /* 15 rows. */
  double *u_6; /* 3 rows. */
  double *x_6; /* 15 rows. */
  double *u_7; /* 3 rows. */
  double *x_7; /* 15 rows. */
  double *u_8; /* 3 rows. */
  double *x_8; /* 15 rows. */
  double *u_9; /* 3 rows. */
  double *x_9; /* 15 rows. */
  double *u_10; /* 3 rows. */
  double *x_10; /* 15 rows. */
  double *u_11; /* 3 rows. */
  double *x_11; /* 15 rows. */
  double *u_12; /* 3 rows. */
  double *x_12; /* 15 rows. */
  double *u_13; /* 3 rows. */
  double *x_13; /* 15 rows. */
  double *u_14; /* 3 rows. */
  double *x_14; /* 15 rows. */
  double *u_15; /* 3 rows. */
  double *x_15; /* 15 rows. */
  double *x_16; /* 15 rows. */
  double *u[16];
  double *x[17];
} Vars;
typedef struct Workspace_t {
  double h[576];
  double s_inv[576];
  double s_inv_z[576];
  double b[240];
  double q[288];
  double rhs[1680];
  double x[1680];
  double *s;
  double *z;
  double *y;
  double lhs_aff[1680];
  double lhs_cc[1680];
  double buffer[1680];
  double buffer2[1680];
  double KKT[3333];
  double L[2325];
  double d[1680];
  double v[1680];
  double d_inv[1680];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_241711181824[1];
  double quad_465695289344[1];
  double quad_391112302592[1];
  double quad_670148681728[1];
  double quad_178111442944[1];
  double quad_199299293184[1];
  double quad_629294600192[1];
  double quad_600158142464[1];
  double quad_244679667712[1];
  double quad_551775350784[1];
  double quad_420535525376[1];
  double quad_480374128640[1];
  double quad_388433166336[1];
  double quad_308192096256[1];
  double quad_495594450944[1];
  double quad_204770549760[1];
  double quad_698454065152[1];
  double quad_610781880320[1];
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
