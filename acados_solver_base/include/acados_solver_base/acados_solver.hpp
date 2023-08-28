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

#ifndef ACADOS_SOLVER_BASE__ACADOS_SOLVER_HPP_
#define ACADOS_SOLVER_BASE__ACADOS_SOLVER_HPP_

#include <iostream>
#include <vector>
#include <exception>

#include "acados_solver_base/visibility_control.h"
#include "acados_solver_base/acados_types.hpp"

// Acados C interface
#include "acados_c/ocp_nlp_interface.h"

namespace acados
{

class AcadosSolver
/**
* @brief Abstract C++ wrapper of generated Acados solver C-code.
*
*/
{
public:
  class Dimensions
    /**
    * @brief Container for the (fixed) dimensions of the imported Acados OCP.
    *
    */
  {
public:
    // Basic dimensions

    /// @brief Differential state vector size
    unsigned int nx;

    /// @brief Algebraic state vector size
    unsigned int nz;

    /// @brief Runtime parameters vector size
    unsigned int np;

    /// @brief Control vector size
    unsigned int nu;

    // Bounds on x and u

    /// @brief Number of state stage bounds
    unsigned int nbx;

    /// @brief Number of initial state stage bounds
    /// @note Typically equal to nx, used to set initial state before calling AcadosSolver::solve().
    unsigned int nbx_0;

    /// @brief Number of terminal state bounds
    unsigned int nbx_N;

    /// @brief Number of control bounds
    /// @note Warning, not defined at stage N.
    unsigned int nbu;

    // Polytope constraints

    /// @brief Number of polytopic constraints
    unsigned int ng;

    /// @brief Number of polytopic constraints at stage N
    unsigned int ng_N;

    // Non-linear constraints

    /// @brief Number of non-linear constraints
    unsigned int nh;

    /// @brief Number of non-linear constraints at stage N
    unsigned int nh_N;

    /// @brief Number of "convex-over-nonlinear" constraints (see Acados documentation)
    unsigned int nphi;

    /// @brief Number of "convex-over-nonlinear" constraints at stage N
    unsigned int nphi_N;

    // Slack-related dimensions

    /// @brief Total number of slack variable
    unsigned int ns;

    /// @brief Total number of slack variable at stage N
    unsigned int ns_N;

    /// @brief Number of slack variables used by state bounds
    unsigned int nsbx;

    /// @brief Number of slack variables used by terminal state bounds
    unsigned int nsbx_N;

    /// @brief Number of slack variables used by control bounds
    unsigned int nsbu;

    /// @brief Number of slack variables used by non-linear constraints
    unsigned int nsh;

    /// @brief Number of slack variables used by (terminal) non-linear constraints
    unsigned int nsh_N;

    /// @brief Number of slack variables used by polytopic constraints
    unsigned int nsg;

    /// @brief Number of slack variables used by (terminal) polytopic constraints
    unsigned int nsg_N;


    /// @brief Number of slack variables used by convex-over-nonlinear constraints
    unsigned int nsphi;

    /// @brief Number of slack variables used by (terminal) convex-over-nonlinear constraints
    unsigned int nsphi_N;

    // Cost ref.
    /// @brief Size of the reference vector
    unsigned int ny;

    /// @brief Size of the reference vector at initial stage
    unsigned int ny_0;

    /// @brief Size of the reference vector at terminal stage
    unsigned int ny_N;

    // Misc.

    /// @brief Dimension of the image of the inner nonlinear function in positive definite constraints. See Acados documentation.
    unsigned int nr;
  };

public:
// Acados solver public API
  AcadosSolver();
  virtual ~AcadosSolver();

  // Solver management
  int init(unsigned int N, double Ts);
  int reset();
  int free_memory();

  // Solve and introspection
  int solve();

  // Setters
  int set_initial_state_values(std::vector<double> & x_0);
  int set_initial_state_values(ValueMap const & x_0_map);

  int set_state_bounds(
    unsigned int stage, std::vector<int> & idxbx, std::vector<double> & lbx,
    std::vector<double> & ubx);
  int set_state_bounds(
    std::vector<int> & idxbx, std::vector<double> & lbx,
    std::vector<double> & ubx);

  int set_control_bounds(
    unsigned int stage, std::vector<int> & idxbu, std::vector<double> & lbu,
    std::vector<double> & ubu);
  int set_control_bounds(
    std::vector<int> & idxbu, std::vector<double> & lbu,
    std::vector<double> & ubu);

  int set_runtime_parameters(unsigned int stage, std::vector<double> & p_i);
  int set_runtime_parameters(unsigned int stage, ValueMap const & p_i_map);
  int set_runtime_parameters(std::vector<double> & p_i);
  int set_runtime_parameters(ValueMap const & p_i_map);

  int initialize_state_values(unsigned int stage, std::vector<double> & x_i);
  int initialize_state_values(unsigned int stage, ValueMap const & x_i_map);
  int initialize_state_values(std::vector<double> & x_i);
  int initialize_state_values(ValueMap const & x_i_map);

  int initialize_control_values(unsigned int stage, std::vector<double> & u_i);
  int initialize_control_values(unsigned int stage, ValueMap const & u_i_map);
  int initialize_control_values(std::vector<double> & u_i);
  int initialize_control_values(ValueMap const & u_i_map);

  // Getters
  std::vector<double> get_state_values(unsigned int stage);
  std::vector<double> get_state_values_as_map(unsigned int stage);

  std::vector<double> get_algebraic_state_values(unsigned int stage);
  std::vector<double> get_algebraic_state_values_as_map(unsigned int stage);

  std::vector<double> get_control_values(unsigned int stage);
  std::vector<double> get_control_values_as_map(unsigned int stage);

  // Getters variable mappings
  const IndexMap & x_index_map() const;
  const IndexMap & z_index_map() const;
  const IndexMap & p_index_map() const;
  const IndexMap & u_index_map() const;

  // Values map utils
  static bool is_map_size_consistent(IndexMap const & index_map, int expected_total_indexes);
  static bool is_map_size_consistent(ValueMap const & values_map, int expected_total_indexes);
  static bool is_values_map_complete(IndexMap const & index_map, ValueMap const & values_map);
  static void fill_vector_from_map(
    IndexMap const & index_map, ValueMap const & values_map,
    unsigned int vector_size, std::vector<double> & values);
  static ValueMap create_map_from_values(
    IndexMap const & index_map,
    std::vector<double> const & values);

  // Problem dimensions and convenience getters for commonly used attributes
  const Dimensions & dims() const;
  unsigned int  nx() const;
  unsigned int  nz() const;
  unsigned int  np() const;
  unsigned int  nu() const;
  unsigned int  N() const;
  double Ts() const;
  std::vector<double> sampling_intervals() const;

protected:
  Dimensions _dims;
  IndexMap _x_index_map;
  IndexMap _z_index_map;
  IndexMap _p_index_map;
  IndexMap _u_index_map;
  // Solver-specific functions
  virtual int create_index_maps() = 0;

  // Imported Acados solver C-code interface ("internal" --> internal use only!)

protected:
  virtual int internal_create_capsule() = 0;
  virtual int internal_create_with_discretization(int n_time_steps, double * new_time_steps) = 0;
  virtual int internal_reset(int reset_qp_solver_mem = 1) = 0;
  virtual int internal_free() = 0;
  virtual int internal_free_capsule() = 0;
  virtual int internal_update_qp_solver_cond_N(int qp_solver_cond_N) = 0;
  virtual int internal_update_params(unsigned int stage, double * value, int np) = 0;
  virtual int internal_update_params_sparse(unsigned int stage, int * idx, double * p, int n_update) = 0;
  virtual int internal_solve() = 0;
  virtual void internal_print_stats() const = 0;

public:
  virtual ocp_nlp_in * get_nlp_in() const = 0;
  virtual ocp_nlp_out * get_nlp_out() const = 0;
  virtual ocp_nlp_out * get_sens_out() const = 0;
  virtual ocp_nlp_solver * get_nlp_solver() const = 0;
  virtual ocp_nlp_config * get_nlp_config() const = 0;
  virtual void * get_nlp_opts() const = 0;
  virtual ocp_nlp_dims * get_nlp_dims() const = 0;
  virtual ocp_nlp_plan_t * get_nlp_plan() const = 0;
  virtual unsigned int get_nlp_np() const = 0;

  // Imported Acados SIM solver C-code interface (for internal use only!)

protected:
  // TODO(tpoignonec) : add sim solver private c-interface

public:
  // TODO(tpoignonec) : add sim solver public c-interface

  // Private own attributes

private:
  bool _is_initialized = false;
  double _Ts = -1;
  // Runtime data
  int _rti_phase = 0;
};

}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_SOLVER_HPP_
