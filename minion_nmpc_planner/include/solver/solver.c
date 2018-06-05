/* Produced by CVXGEN, 2016-12-22 06:28:39 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 576; i++)
    gap += work.z[i]*work.s[i];
  return gap;
}
void set_defaults(void) {
  settings.resid_tol = 1e-6;
  settings.eps = 1e-4;
  settings.max_iters = 25;
  settings.refine_steps = 1;
  settings.s_init = 1;
  settings.z_init = 1;
  settings.debug = 0;
  settings.verbose = 1;
  settings.verbose_refinement = 0;
  settings.better_start = 1;
  settings.kkt_reg = 1e-7;
}
void setup_pointers(void) {
  work.y = work.x + 288;
  work.s = work.x + 528;
  work.z = work.x + 1104;
  vars.u_0 = work.x + 0;
  vars.u_1 = work.x + 3;
  vars.u_2 = work.x + 6;
  vars.u_3 = work.x + 9;
  vars.u_4 = work.x + 12;
  vars.u_5 = work.x + 15;
  vars.u_6 = work.x + 18;
  vars.u_7 = work.x + 21;
  vars.u_8 = work.x + 24;
  vars.u_9 = work.x + 27;
  vars.u_10 = work.x + 30;
  vars.u_11 = work.x + 33;
  vars.u_12 = work.x + 36;
  vars.u_13 = work.x + 39;
  vars.u_14 = work.x + 42;
  vars.u_15 = work.x + 45;
  vars.x_1 = work.x + 48;
  vars.x_2 = work.x + 63;
  vars.x_3 = work.x + 78;
  vars.x_4 = work.x + 93;
  vars.x_5 = work.x + 108;
  vars.x_6 = work.x + 123;
  vars.x_7 = work.x + 138;
  vars.x_8 = work.x + 153;
  vars.x_9 = work.x + 168;
  vars.x_10 = work.x + 183;
  vars.x_11 = work.x + 198;
  vars.x_12 = work.x + 213;
  vars.x_13 = work.x + 228;
  vars.x_14 = work.x + 243;
  vars.x_15 = work.x + 258;
  vars.x_16 = work.x + 273;
}
void setup_indexed_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  params.x[0] = params.x_0;
  params.xr[0] = params.xr_0;
  params.xr[1] = params.xr_1;
  params.xr[2] = params.xr_2;
  params.xr[3] = params.xr_3;
  params.xr[4] = params.xr_4;
  params.xr[5] = params.xr_5;
  params.xr[6] = params.xr_6;
  params.xr[7] = params.xr_7;
  params.xr[8] = params.xr_8;
  params.xr[9] = params.xr_9;
  params.xr[10] = params.xr_10;
  params.xr[11] = params.xr_11;
  params.xr[12] = params.xr_12;
  params.xr[13] = params.xr_13;
  params.xr[14] = params.xr_14;
  params.xr[15] = params.xr_15;
}
void setup_indexed_optvars(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  vars.u[0] = vars.u_0;
  vars.u[1] = vars.u_1;
  vars.x[1] = vars.x_1;
  vars.u[2] = vars.u_2;
  vars.x[2] = vars.x_2;
  vars.u[3] = vars.u_3;
  vars.x[3] = vars.x_3;
  vars.u[4] = vars.u_4;
  vars.x[4] = vars.x_4;
  vars.u[5] = vars.u_5;
  vars.x[5] = vars.x_5;
  vars.u[6] = vars.u_6;
  vars.x[6] = vars.x_6;
  vars.u[7] = vars.u_7;
  vars.x[7] = vars.x_7;
  vars.u[8] = vars.u_8;
  vars.x[8] = vars.x_8;
  vars.u[9] = vars.u_9;
  vars.x[9] = vars.x_9;
  vars.u[10] = vars.u_10;
  vars.x[10] = vars.x_10;
  vars.u[11] = vars.u_11;
  vars.x[11] = vars.x_11;
  vars.u[12] = vars.u_12;
  vars.x[12] = vars.x_12;
  vars.u[13] = vars.u_13;
  vars.x[13] = vars.x_13;
  vars.u[14] = vars.u_14;
  vars.x[14] = vars.x_14;
  vars.u[15] = vars.u_15;
  vars.x[15] = vars.x_15;
  vars.x[16] = vars.x_16;
}
void setup_indexing(void) {
  setup_pointers();
  setup_indexed_params();
  setup_indexed_optvars();
}
void set_start(void) {
  int i;
  for (i = 0; i < 288; i++)
    work.x[i] = 0;
  for (i = 0; i < 240; i++)
    work.y[i] = 0;
  for (i = 0; i < 576; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;
  for (i = 0; i < 576; i++)
    work.z[i] = settings.z_init;
}
double eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);
  objv = 0;
  for (i = 0; i < 288; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 288; i++)
    objv += work.q[i]*work.x[i];
  objv += work.quad_241711181824[0]+work.quad_465695289344[0]+work.quad_241711181824[0]+work.quad_391112302592[0]+work.quad_241711181824[0]+work.quad_670148681728[0]+work.quad_241711181824[0]+work.quad_178111442944[0]+work.quad_241711181824[0]+work.quad_199299293184[0]+work.quad_241711181824[0]+work.quad_629294600192[0]+work.quad_241711181824[0]+work.quad_600158142464[0]+work.quad_241711181824[0]+work.quad_244679667712[0]+work.quad_241711181824[0]+work.quad_551775350784[0]+work.quad_241711181824[0]+work.quad_420535525376[0]+work.quad_241711181824[0]+work.quad_480374128640[0]+work.quad_241711181824[0]+work.quad_388433166336[0]+work.quad_241711181824[0]+work.quad_308192096256[0]+work.quad_241711181824[0]+work.quad_495594450944[0]+work.quad_241711181824[0]+work.quad_204770549760[0]+work.quad_241711181824[0]+work.quad_698454065152[0]+work.quad_610781880320[0];
  return objv;
}
void fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 288;
  r3 = work.rhs + 864;
  r4 = work.rhs + 1440;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 288; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 288; i++)
    r1[i] -= work.buffer[i] + work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 576; i++)
    r2[i] = -work.z[i];
  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 576; i++)
    r3[i] += -work.s[i] + work.h[i];
  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 240; i++)
    r4[i] += work.b[i];
}
void fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = work.rhs + 288;
  ds_aff = work.lhs_aff + 288;
  dz_aff = work.lhs_aff + 864;
  mu = 0;
  for (i = 0; i < 576; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 576; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 576; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 576; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.001736111111111111;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 288; i++)
    work.rhs[i] = 0;
  for (i = 864; i < 1680; i++)
    work.rhs[i] = 0;
  for (i = 0; i < 576; i++)
    r2[i] = work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void refine(double *target, double *var) {
  int i, j;
  double *residual = work.buffer;
  double norm2;
  double *new_var = work.buffer2;
  for (j = 0; j < settings.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 1680; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 1680; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 1680; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG(work.buffer, work.x);
  /* Add -s + h. */
  for (i = 0; i < 576; i++)
    work.buffer[i] += -work.s[i] + work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 576; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
double calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA(work.buffer, work.x);
  /* Add +b. */
  for (i = 0; i < 240; i++)
    work.buffer[i] += work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 240; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
void better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 576; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();
  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = work.lhs_aff;
  s = work.lhs_aff + 288;
  z = work.lhs_aff + 864;
  y = work.lhs_aff + 1440;
  /* Just set x and y as is. */
  for (i = 0; i < 288; i++)
    work.x[i] = x[i];
  for (i = 0; i < 240; i++)
    work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 576; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 576; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 576; i++)
      work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 576; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 576; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 576; i++)
      work.z[i] = z[i] + alpha;
  }
}
void fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 288;
  r3 = work.rhs + 864;
  r4 = work.rhs + 1440;
  for (i = 0; i < 288; i++)
    r1[i] = -work.q[i];
  for (i = 0; i < 576; i++)
    r2[i] = 0;
  for (i = 0; i < 576; i++)
    r3[i] = work.h[i];
  for (i = 0; i < 240; i++)
    r4[i] = work.b[i];
}
long solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  work.converged = 0;
  setup_pointers();
  pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq();
  fillh();
  fillb();
  if (settings.better_start)
    better_start();
  else
    set_start();
  for (iter = 0; iter < settings.max_iters; iter++) {
    for (i = 0; i < 576; i++) {
      work.s_inv[i] = 1.0 / work.s[i];
      work.s_inv_z[i] = work.s_inv[i]*work.z[i];
    }
    work.block_33[0] = 0;
    fill_KKT();
    ldl_factor();
    /* Affine scaling directions. */
    fillrhs_aff();
    ldl_solve(work.rhs, work.lhs_aff);
    refine(work.rhs, work.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc();
    ldl_solve(work.rhs, work.lhs_cc);
    refine(work.rhs, work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 1680; i++)
      work.lhs_aff[i] += work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 288;
    dz = work.lhs_aff + 864;
    dy = work.lhs_aff + 1440;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 576; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 576; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 288; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 576; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 576; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 240; i++)
      work.y[i] += alpha*dy[i];
    work.gap = eval_gap();
    work.eq_resid_squared = calc_eq_resid_squared();
    work.ineq_resid_squared = calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose) {
      work.optval = eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, work.optval, work.gap, sqrt(work.eq_resid_squared),
          sqrt(work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (work.gap < settings.eps)
        && (work.eq_resid_squared <= settings.resid_tol*settings.resid_tol)
        && (work.ineq_resid_squared <= settings.resid_tol*settings.resid_tol)
       ) {
      work.converged = 1;
      work.optval = eval_objv();
      return iter+1;
    }
  }
  return iter;
}
