/* This file was automatically generated by CasADi 3.6.7.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) mock_acados_solver_impl_dae_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[18] = {9, 9, 0, 0, 3, 3, 5, 5, 5, 5, 5, 6, 1, 3, 8, 1, 3, 1};

/* mock_acados_solver_impl_dae_hess:(i0[4],i1[4],i2,i3[],i4[4],i5[],i6[2])->(o0[9x9,6nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][1] : 0;
  a1=cos(a0);
  a2=9.8100000000000005e+00;
  a3=arg[6]? arg[6][0] : 0;
  a4=arg[6]? arg[6][1] : 0;
  a5=(a3+a4);
  a5=(a2*a5);
  a6=arg[4]? arg[4][3] : 0;
  a7=8.0000000000000004e-01;
  a3=(a3+a4);
  a8=cos(a0);
  a9=(a4*a8);
  a10=(a9*a8);
  a3=(a3-a10);
  a10=(a7*a3);
  a11=(a6/a10);
  a12=(a11/a10);
  a13=sin(a0);
  a14=(a4*a13);
  a15=(a8*a14);
  a16=(a9*a13);
  a15=(a15+a16);
  a16=(a7*a15);
  a12=(a12*a16);
  a17=(a5*a12);
  a18=arg[0]? arg[0][3] : 0;
  a19=(a18*a11);
  a20=(a18*a19);
  a21=(a7*a4);
  a22=(a21*a13);
  a23=(a20*a22);
  a24=(a21*a8);
  a25=(a18*a12);
  a26=(a18*a25);
  a27=(a24*a26);
  a23=(a23+a27);
  a17=(a17-a23);
  a23=(a7*a4);
  a27=arg[4]? arg[4][2] : 0;
  a28=(a27/a3);
  a29=(a28/a3);
  a29=(a29*a15);
  a30=(a18*a29);
  a31=(a18*a30);
  a31=(a23*a31);
  a17=(a17-a31);
  a2=(a2*a4);
  a31=(a2*a13);
  a32=(a28*a31);
  a33=(a2*a8);
  a34=(a33*a29);
  a32=(a32+a34);
  a17=(a17+a32);
  a1=(a1*a17);
  a17=(a24*a20);
  a32=(a5*a11);
  a17=(a17-a32);
  a32=(a18*a28);
  a34=(a18*a32);
  a34=(a23*a34);
  a17=(a17+a34);
  a34=(a33*a28);
  a17=(a17-a34);
  a34=sin(a0);
  a17=(a17*a34);
  a1=(a1-a17);
  a17=sin(a0);
  a34=(a17*a20);
  a34=(a21*a34);
  a35=arg[2]? arg[2][0] : 0;
  a36=(a35*a11);
  a34=(a34-a36);
  a36=(a35*a8);
  a37=(a24*a17);
  a38=(a37*a18);
  a39=(a38*a18);
  a36=(a36-a39);
  a39=(a5*a17);
  a36=(a36+a39);
  a36=(a36/a10);
  a39=(a36/a10);
  a40=(a39*a6);
  a40=(a7*a40);
  a41=(a33*a17);
  a42=(a23*a17);
  a43=(a42*a18);
  a44=(a43*a18);
  a41=(a41-a44);
  a41=(a41+a35);
  a41=(a41/a3);
  a44=(a41/a3);
  a45=(a44*a27);
  a40=(a40+a45);
  a45=(a9*a40);
  a34=(a34-a45);
  a45=(a8*a40);
  a45=(a4*a45);
  a34=(a34-a45);
  a45=(a17*a28);
  a45=(a2*a45);
  a34=(a34-a45);
  a45=cos(a0);
  a34=(a34*a45);
  a45=sin(a0);
  a0=cos(a0);
  a20=(a20*a0);
  a26=(a17*a26);
  a20=(a20-a26);
  a21=(a21*a20);
  a20=(a35*a12);
  a21=(a21+a20);
  a5=(a5*a0);
  a35=(a35*a13);
  a24=(a24*a0);
  a22=(a17*a22);
  a24=(a24-a22);
  a22=(a18*a24);
  a20=(a18*a22);
  a35=(a35+a20);
  a5=(a5-a35);
  a5=(a5/a10);
  a36=(a36/a10);
  a36=(a36*a16);
  a5=(a5-a36);
  a5=(a5/a10);
  a39=(a39/a10);
  a39=(a39*a16);
  a5=(a5-a39);
  a6=(a6*a5);
  a7=(a7*a6);
  a33=(a33*a0);
  a31=(a17*a31);
  a33=(a33-a31);
  a23=(a23*a0);
  a31=(a18*a23);
  a18=(a18*a31);
  a33=(a33-a18);
  a33=(a33/a3);
  a41=(a41/a3);
  a41=(a41*a15);
  a33=(a33-a41);
  a33=(a33/a3);
  a44=(a44/a3);
  a44=(a44*a15);
  a33=(a33-a44);
  a27=(a27*a33);
  a7=(a7+a27);
  a9=(a9*a7);
  a14=(a40*a14);
  a9=(a9-a14);
  a21=(a21-a9);
  a7=(a8*a7);
  a40=(a40*a13);
  a7=(a7-a40);
  a4=(a4*a7);
  a21=(a21-a4);
  a0=(a28*a0);
  a17=(a17*a29);
  a0=(a0-a17);
  a2=(a2*a0);
  a21=(a21-a2);
  a45=(a45*a21);
  a34=(a34+a45);
  a1=(a1-a34);
  if (res[0]!=0) res[0][0]=a1;
  a22=(a11*a22);
  a38=(a38*a12);
  a22=(a22-a38);
  a19=(a19*a24);
  a25=(a37*a25);
  a19=(a19-a25);
  a22=(a22+a19);
  a31=(a28*a31);
  a43=(a43*a29);
  a31=(a31-a43);
  a22=(a22+a31);
  a32=(a32*a23);
  a30=(a42*a30);
  a32=(a32-a30);
  a22=(a22+a32);
  if (res[0]!=0) res[0][1]=a22;
  a13=(a11*a13);
  a8=(a8*a12);
  a13=(a13+a8);
  a13=(a13+a29);
  if (res[0]!=0) res[0][2]=a13;
  if (res[0]!=0) res[0][3]=a22;
  a22=(a11*a37);
  a37=(a37*a11);
  a22=(a22+a37);
  a37=(a28*a42);
  a22=(a22+a37);
  a42=(a42*a28);
  a22=(a22+a42);
  if (res[0]!=0) res[0][4]=a22;
  if (res[0]!=0) res[0][5]=a13;
  return 0;
}

CASADI_SYMBOL_EXPORT int mock_acados_solver_impl_dae_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int mock_acados_solver_impl_dae_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int mock_acados_solver_impl_dae_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void mock_acados_solver_impl_dae_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int mock_acados_solver_impl_dae_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void mock_acados_solver_impl_dae_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void mock_acados_solver_impl_dae_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void mock_acados_solver_impl_dae_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int mock_acados_solver_impl_dae_hess_n_in(void) { return 7;}

CASADI_SYMBOL_EXPORT casadi_int mock_acados_solver_impl_dae_hess_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real mock_acados_solver_impl_dae_hess_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* mock_acados_solver_impl_dae_hess_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    case 6: return "i6";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* mock_acados_solver_impl_dae_hess_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* mock_acados_solver_impl_dae_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s0;
    case 5: return casadi_s2;
    case 6: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* mock_acados_solver_impl_dae_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int mock_acados_solver_impl_dae_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int mock_acados_solver_impl_dae_hess_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
