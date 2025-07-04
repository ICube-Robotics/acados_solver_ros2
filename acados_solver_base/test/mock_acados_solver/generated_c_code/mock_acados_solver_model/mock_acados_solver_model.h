/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef mock_acados_solver_MODEL
#define mock_acados_solver_MODEL

#ifdef __cplusplus
extern "C" {
#endif


  
// implicit ODE: function
int mock_acados_solver_impl_dae_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int mock_acados_solver_impl_dae_fun_work(int *, int *, int *, int *);
const int *mock_acados_solver_impl_dae_fun_sparsity_in(int);
const int *mock_acados_solver_impl_dae_fun_sparsity_out(int);
int mock_acados_solver_impl_dae_fun_n_in(void);
int mock_acados_solver_impl_dae_fun_n_out(void);

// implicit ODE: function + jacobians
int mock_acados_solver_impl_dae_fun_jac_x_xdot_z(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int mock_acados_solver_impl_dae_fun_jac_x_xdot_z_work(int *, int *, int *, int *);
const int *mock_acados_solver_impl_dae_fun_jac_x_xdot_z_sparsity_in(int);
const int *mock_acados_solver_impl_dae_fun_jac_x_xdot_z_sparsity_out(int);
int mock_acados_solver_impl_dae_fun_jac_x_xdot_z_n_in(void);
int mock_acados_solver_impl_dae_fun_jac_x_xdot_z_n_out(void);

// implicit ODE: jacobians only
int mock_acados_solver_impl_dae_jac_x_xdot_u_z(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int mock_acados_solver_impl_dae_jac_x_xdot_u_z_work(int *, int *, int *, int *);
const int *mock_acados_solver_impl_dae_jac_x_xdot_u_z_sparsity_in(int);
const int *mock_acados_solver_impl_dae_jac_x_xdot_u_z_sparsity_out(int);
int mock_acados_solver_impl_dae_jac_x_xdot_u_z_n_in(void);
int mock_acados_solver_impl_dae_jac_x_xdot_u_z_n_out(void);

// implicit ODE - for lifted_irk
int mock_acados_solver_impl_dae_fun_jac_x_xdot_u(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int mock_acados_solver_impl_dae_fun_jac_x_xdot_u_work(int *, int *, int *, int *);
const int *mock_acados_solver_impl_dae_fun_jac_x_xdot_u_sparsity_in(int);
const int *mock_acados_solver_impl_dae_fun_jac_x_xdot_u_sparsity_out(int);
int mock_acados_solver_impl_dae_fun_jac_x_xdot_u_n_in(void);
int mock_acados_solver_impl_dae_fun_jac_x_xdot_u_n_out(void);
// implicit ODE - hessian
int mock_acados_solver_impl_dae_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int mock_acados_solver_impl_dae_hess_work(int *, int *, int *, int *);
const int *mock_acados_solver_impl_dae_hess_sparsity_in(int);
const int *mock_acados_solver_impl_dae_hess_sparsity_out(int);
int mock_acados_solver_impl_dae_hess_n_in(void);
int mock_acados_solver_impl_dae_hess_n_out(void);
    
  



#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // mock_acados_solver_MODEL
