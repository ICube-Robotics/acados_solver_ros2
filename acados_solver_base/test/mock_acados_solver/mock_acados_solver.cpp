// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Thibault Poignonec (tpoignonec@unistra.fr)

#include <iostream>
#include <string>

#include "acados_solver_base/acados_solver.hpp"
#include "mock_acados_solver.hpp"

// Include auto-generated Acados C-code
#include "generated_c_code/acados_solver_mock_acados_solver.h"
#include "generated_c_code/acados_sim_solver_mock_acados_solver.h"


namespace mock_acados_solver_test
{

using namespace acados;

// Constructor
MockAcadosSolver::MockAcadosSolver()
{
  // Fill fixed problem dimensions
  // Main variables
  _dims.nx = MOCK_ACADOS_SOLVER_NX;
  _dims.nz = MOCK_ACADOS_SOLVER_NZ;
  _dims.np = MOCK_ACADOS_SOLVER_NP;
  _dims.nu = MOCK_ACADOS_SOLVER_NU;
  // Constraints
  _dims.nbx = MOCK_ACADOS_SOLVER_NBX;
  _dims.nbx_0 = MOCK_ACADOS_SOLVER_NBX0;
  _dims.nbx_N = MOCK_ACADOS_SOLVER_NBXN;
  _dims.nbu = MOCK_ACADOS_SOLVER_NBU;
  _dims.ng = MOCK_ACADOS_SOLVER_NG;
  _dims.ng_N = MOCK_ACADOS_SOLVER_NGN;
  // Non-linear constraints
  _dims.nh = MOCK_ACADOS_SOLVER_NH;
  _dims.nh_N = MOCK_ACADOS_SOLVER_NHN;
  _dims.nphi = MOCK_ACADOS_SOLVER_NPHI;
  _dims.nphi_N = MOCK_ACADOS_SOLVER_NPHIN;
  _dims.nr = MOCK_ACADOS_SOLVER_NR;
  // Slack
  _dims.ns = MOCK_ACADOS_SOLVER_NS;
  _dims.ns_N = MOCK_ACADOS_SOLVER_NSN;
  _dims.nsbx = MOCK_ACADOS_SOLVER_NSBX;
  _dims.nsbx_N = MOCK_ACADOS_SOLVER_NSBXN;
  _dims.nsbu = MOCK_ACADOS_SOLVER_NSBU;
  _dims.nsh = MOCK_ACADOS_SOLVER_NSH;
  _dims.nsh_N = MOCK_ACADOS_SOLVER_NSHN;
  _dims.nsg = MOCK_ACADOS_SOLVER_NSG;
  _dims.nsg_N = MOCK_ACADOS_SOLVER_NSGN;
  _dims.nsphi = MOCK_ACADOS_SOLVER_NSPHI;
  _dims.nsphi_N = MOCK_ACADOS_SOLVER_NSPHIN;
  // Corst reference
  _dims.ny = MOCK_ACADOS_SOLVER_NY;
  _dims.ny_0 = MOCK_ACADOS_SOLVER_NY0;
  _dims.ny_N = MOCK_ACADOS_SOLVER_NYN;
}
MockAcadosSolver::~MockAcadosSolver()
{
  free_memory();
}

int MockAcadosSolver::create_index_maps()
{
  // Clear all
  _x_index_map.clear();
  _z_index_map.clear();
  _p_index_map.clear();
  _u_index_map.clear();

  // Differential state variables
  _x_index_map[std::string("p")] = {0};
  _x_index_map[std::string("p_dot")] = {1};
  _x_index_map[std::string("theta")] = {2};
  _x_index_map[std::string("theta_dot")] = {3};

  // Algebraic state variables

  // Runtime parameters
  _p_index_map[std::string("mass_cart")] = {0};
  _p_index_map[std::string("mass_ball")] = {1};

  // Control variables
  _u_index_map[std::string("f")] = {0};

  return 0;
}

int MockAcadosSolver::internal_create_capsule()
{
  _capsule = mock_acados_solver_acados_create_capsule();
  _capsule_sim = mock_acados_solver_acados_sim_solver_create_capsule();
  return (_capsule) ? 0 : -1;
}
int MockAcadosSolver::internal_create_with_discretization(
  int n_time_steps,
  double * new_time_steps)
{
  int ret = mock_acados_solver_acados_create_with_discretization(_capsule, n_time_steps, new_time_steps);
  ret += mock_acados_solver_acados_sim_create(_capsule_sim);
  return ret;
}
int MockAcadosSolver::internal_reset(int reset_qp_solver_mem)
{
  return mock_acados_solver_acados_reset(_capsule, reset_qp_solver_mem);
}
int MockAcadosSolver::internal_free()
{
  int ret = mock_acados_solver_acados_free(_capsule);
  ret += mock_acados_solver_acados_sim_free(_capsule_sim);
  return ret;
}
int MockAcadosSolver::internal_free_capsule()
{
  int ret = mock_acados_solver_acados_free_capsule(_capsule);
  ret += mock_acados_solver_acados_sim_solver_free_capsule(_capsule_sim);
  return ret;
}

int MockAcadosSolver::internal_update_qp_solver_cond_N(int qp_solver_cond_N)
{
  return mock_acados_solver_acados_update_qp_solver_cond_N(_capsule, qp_solver_cond_N);
}
int MockAcadosSolver::internal_update_params(unsigned int stage, double * value, int np)
{
  return mock_acados_solver_acados_update_params(_capsule, static_cast<int>(stage), value, np);
}
int MockAcadosSolver::internal_update_params_sparse(unsigned int stage, int * idx, double * p, int n_update)
{
  return mock_acados_solver_acados_update_params_sparse(_capsule, static_cast<int>(stage), idx, p, n_update);
}
int MockAcadosSolver::internal_solve()
{
  return mock_acados_solver_acados_solve(_capsule);
}
void MockAcadosSolver::internal_print_stats() const
{
  return mock_acados_solver_acados_print_stats(_capsule);
}

ocp_nlp_in * MockAcadosSolver::get_nlp_in() const
{
  return mock_acados_solver_acados_get_nlp_in(_capsule);
}

ocp_nlp_out * MockAcadosSolver::get_nlp_out() const
{
  return mock_acados_solver_acados_get_nlp_out(_capsule);
}

ocp_nlp_out * MockAcadosSolver::get_sens_out() const
{
  return mock_acados_solver_acados_get_sens_out(_capsule);
}
ocp_nlp_solver * MockAcadosSolver::get_nlp_solver() const
{
  return mock_acados_solver_acados_get_nlp_solver(_capsule);
}
ocp_nlp_config * MockAcadosSolver::get_nlp_config() const
{
  return mock_acados_solver_acados_get_nlp_config(_capsule);
}
void * MockAcadosSolver::get_nlp_opts() const
{
  return mock_acados_solver_acados_get_nlp_opts(_capsule);
}
ocp_nlp_dims * MockAcadosSolver::get_nlp_dims() const
{
  return mock_acados_solver_acados_get_nlp_dims(_capsule);
}
ocp_nlp_plan_t * MockAcadosSolver::get_nlp_plan() const
{
  return mock_acados_solver_acados_get_nlp_plan(_capsule);
}
unsigned int MockAcadosSolver::get_nlp_np() const
{
  return _capsule->nlp_np;
}

int MockAcadosSolver::internal_simulate(
  double dt /* Time step */,
  double * x0 /* Differential state initial value */,
  double * u0 /* Applied controls */,
  double * p /* Runtime parameters */,
  double * x_next /* Differential state next value */,
  double * z_next /* Algebraic state next value */)
{
  if (dt <= 0.0) {
    std::cerr
      << "Error in MockAcadosSolver::simulate: Invalid time step, got "
      << dt << std::endl;
    return 10; // Error: Invalid time step
  }
  if (x0 == nullptr || u0 == nullptr || p == nullptr || x_next == nullptr || z_next == nullptr) {
    return 11; // Error: Null pointer passed to simulate function
  }
  if (_capsule_sim == nullptr) {
    return 12; // Error: Simulation capsule not created
  }

  // Allocate return flag
  int ret = 0;

  // Set state and control initial values
  sim_in_set(
    mock_acados_solver_acados_get_sim_config(_capsule_sim),
    mock_acados_solver_acados_get_sim_dims(_capsule_sim),
    mock_acados_solver_acados_get_sim_in(_capsule_sim),
    "x", x0
  );
  sim_in_set(
    mock_acados_solver_acados_get_sim_config(_capsule_sim),
    mock_acados_solver_acados_get_sim_dims(_capsule_sim),
    mock_acados_solver_acados_get_sim_in(_capsule_sim),
    "u", u0
  );

  // Set parameters
  ret = mock_acados_solver_acados_sim_update_params(_capsule_sim, p, _dims.np);
  if (ret != 0) {
    std::cerr << "Error in MockAcadosSolver::simulate: Failed to update parameters." << std::endl;
    return ret;
  }

  // Simulate
  ret = mock_acados_solver_acados_sim_solve(_capsule_sim);
  if (ret != 0) {
    std::cerr << "Error in MockAcadosSolver::simulate: Simulation solve failed." << std::endl;
  }

  // Get next state and algebraic state
  sim_out_get(
    mock_acados_solver_acados_get_sim_config(_capsule_sim),
    mock_acados_solver_acados_get_sim_dims(_capsule_sim),
    mock_acados_solver_acados_get_sim_out(_capsule_sim),
    "x", x_next
  );
  sim_out_get(
    mock_acados_solver_acados_get_sim_config(_capsule_sim),
    mock_acados_solver_acados_get_sim_dims(_capsule_sim),
    mock_acados_solver_acados_get_sim_out(_capsule_sim),
    "z", z_next
  );
  return ret;
}

}  // namespace mock_acados_solver_test

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(mock_acados_solver_test::MockAcadosSolver, acados::AcadosSolver)