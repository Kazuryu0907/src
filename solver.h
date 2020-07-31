/* Produced by CVXGEN, 2020-07-30 08:56:48 -0400.  */
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
  double a[1];
  double xr_0[3];
  double *A1;
  double b[1];
  double ur_0[3];
  double xr_1[3];
  double ur_1[3];
  double xr_2[3];
  double ur_2[3];
  double xr_3[3];
  double ur_3[3];
  double xr_4[3];
  double ur_4[3];
  double xr_5[3];
  double ur_5[3];
  double xr_6[3];
  double ur_6[3];
  double xr_7[3];
  double ur_7[3];
  double xr_8[3];
  double ur_8[3];
  double xr_9[3];
  double ur_9[3];
  double xr_10[3];
  double ur_10[3];
  double xr_11[3];
  double ur_11[3];
  double xr_12[3];
  double ur_12[3];
  double xr_13[3];
  double ur_13[3];
  double xr_14[3];
  double ur_14[3];
  double xr_15[3];
  double ur_15[3];
  double A[3];
  double dt[1];
  double *B;
  double maxu[3];
  double xzero[3];
  double Theta1[3];
  double maxthetaerror[1];
  double *xr[16];
  double *ur[16];
} Params;
typedef struct Vars_t {
  double *x_0; /* 3 rows. */
  double *u_0; /* 3 rows. */
  double *x_1; /* 3 rows. */
  double *u_1; /* 3 rows. */
  double *x_2; /* 3 rows. */
  double *u_2; /* 3 rows. */
  double *x_3; /* 3 rows. */
  double *u_3; /* 3 rows. */
  double *x_4; /* 3 rows. */
  double *u_4; /* 3 rows. */
  double *x_5; /* 3 rows. */
  double *u_5; /* 3 rows. */
  double *x_6; /* 3 rows. */
  double *u_6; /* 3 rows. */
  double *x_7; /* 3 rows. */
  double *u_7; /* 3 rows. */
  double *x_8; /* 3 rows. */
  double *u_8; /* 3 rows. */
  double *x_9; /* 3 rows. */
  double *u_9; /* 3 rows. */
  double *x_10; /* 3 rows. */
  double *u_10; /* 3 rows. */
  double *x_11; /* 3 rows. */
  double *u_11; /* 3 rows. */
  double *x_12; /* 3 rows. */
  double *u_12; /* 3 rows. */
  double *x_13; /* 3 rows. */
  double *u_13; /* 3 rows. */
  double *x_14; /* 3 rows. */
  double *u_14; /* 3 rows. */
  double *x_15; /* 3 rows. */
  double *u_15; /* 3 rows. */
  double *t_01; /* 3 rows. */
  double *t_02; /* 3 rows. */
  double *t_03; /* 3 rows. */
  double *t_04; /* 3 rows. */
  double *t_05; /* 3 rows. */
  double *t_06; /* 3 rows. */
  double *t_07; /* 3 rows. */
  double *t_08; /* 3 rows. */
  double *t_09; /* 3 rows. */
  double *t_10; /* 3 rows. */
  double *t_11; /* 3 rows. */
  double *t_12; /* 3 rows. */
  double *t_13; /* 3 rows. */
  double *t_14; /* 3 rows. */
  double *t_15; /* 3 rows. */
  double *t_16; /* 3 rows. */
  double *t_17; /* 1 rows. */
  double *t_18; /* 1 rows. */
  double *t_19; /* 1 rows. */
  double *t_20; /* 1 rows. */
  double *t_21; /* 1 rows. */
  double *t_22; /* 1 rows. */
  double *t_23; /* 1 rows. */
  double *t_24; /* 1 rows. */
  double *t_25; /* 1 rows. */
  double *t_26; /* 1 rows. */
  double *t_27; /* 1 rows. */
  double *t_28; /* 1 rows. */
  double *t_29; /* 1 rows. */
  double *t_30; /* 1 rows. */
  double *t_31; /* 1 rows. */
  double *t_32; /* 1 rows. */
  double *x_16; /* 3 rows. */
  double *x[17];
  double *u[16];
} Vars;
typedef struct Workspace_t {
  double h[192];
  double s_inv[192];
  double s_inv_z[192];
  double b[51];
  double q[163];
  double rhs[598];
  double x[598];
  double *s;
  double *z;
  double *y;
  double lhs_aff[598];
  double lhs_cc[598];
  double buffer[598];
  double buffer2[598];
  double KKT[1347];
  double L[1072];
  double d[598];
  double v[598];
  double d_inv[598];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_842467643392[1];
  double quad_228651425792[1];
  double quad_435365027840[1];
  double quad_44995719168[1];
  double quad_883793121280[1];
  double quad_293350719488[1];
  double quad_734795079680[1];
  double quad_223196893184[1];
  double quad_607042646016[1];
  double quad_472993533952[1];
  double quad_897988612096[1];
  double quad_800194449408[1];
  double quad_866568097792[1];
  double quad_763385561088[1];
  double quad_882010787840[1];
  double quad_396827062272[1];
  double quad_866363117568[1];
  double quad_213032898560[1];
  double quad_53827780608[1];
  double quad_266531401728[1];
  double quad_752900558848[1];
  double quad_325904396288[1];
  double quad_544070639616[1];
  double quad_809677381632[1];
  double quad_701018628096[1];
  double quad_468389339136[1];
  double quad_401761996800[1];
  double quad_776522235904[1];
  double quad_433217142784[1];
  double quad_725676359680[1];
  double quad_90731343872[1];
  double quad_239712845824[1];
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
#ifdef __cplusplus
extern "C" {
#endif
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
#ifdef __cplusplus
}
#endif
/* Function definitions in testsolver.c: */
//int main(int argc, char **argv);
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
