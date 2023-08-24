#ifndef ACADOS_SOLVER__ACADOS_SOLVER_HPP_
#define ACADOS_SOLVER__ACADOS_SOLVER_HPP_

#include "acados_solver_base/visibility_control.h"
#include "acados_solver_base/acados_types.hpp"
#include "acados_solver_base/acados_solver_dimensions.hpp"

// Acados C interface
#include "acados_c/ocp_nlp_interface.h"

// Misc.
#include <iostream>
#include <exception>

namespace acados
{

class AcadosSolver
{
// Acados solver public API

public:
  using Dimensions = AcadosSolverDimensions;

  AcadosSolver();
  virtual ~AcadosSolver();

  // Solver management
  int init(int N, double Ts);
  int reset();
  int free_memory();

  // Solve and introspection
  int solve();

  // Setters
  int set_initial_state_values(std::vector<double> & x_0);
  int set_initial_state_values(ValueMap const & x_0_map);

  int set_state_bounds(
    int stage, std::vector<int> & idxbx, std::vector<double> & lbx,
    std::vector<double> & ubx);
  int set_state_bounds(
    std::vector<int> & idxbx, std::vector<double> & lbx,
    std::vector<double> & ubx);

  int set_control_bounds(
    int stage, std::vector<int> & idxbu, std::vector<double> & lbu,
    std::vector<double> & ubu);
  int set_control_bounds(
    std::vector<int> & idxbu, std::vector<double> & lbu,
    std::vector<double> & ubu);

  int set_runtime_parameters(int stage, std::vector<double> & p_i);
  int set_runtime_parameters(int stage, ValueMap const & p_i_map);
  int set_runtime_parameters(std::vector<double> & p_i);
  int set_runtime_parameters(ValueMap const & p_i_map);

  int initialize_state_values(int stage, std::vector<double> & x_i);
  int initialize_state_values(int stage, ValueMap const & x_i_map);
  int initialize_state_values(std::vector<double> & x_i);
  int initialize_state_values(ValueMap const & x_i_map);

  int initialize_control_values(int stage, std::vector<double> & u_i);
  int initialize_control_values(int stage, ValueMap const & u_i_map);
  int initialize_control_values(std::vector<double> & u_i);
  int initialize_control_values(ValueMap const & u_i_map);

  // Getters
  std::vector<double> get_state_values(int stage);
  std::vector<double> get_state_values_as_map(int stage);

  std::vector<double> get_algebraic_state_values(int stage);
  std::vector<double> get_algebraic_state_values_as_map(int stage);

  std::vector<double> get_control_values(int stage);
  std::vector<double> get_control_values_as_map(int stage);

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

  // Problem dimensions and conveniance getters for commonly used attributs
  const Dimensions & dims() const;
  int nx() const;
  int nz() const;
  int np() const;
  int nu() const;
  int N() const;
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
  virtual int internal_update_params(int stage, double * value, int np) = 0;
  virtual int internal_update_params_sparse(int stage, int * idx, double * p, int n_update) = 0;
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
  // TODO

public:
  // TODO

  // Private own attributs

private:
  bool _is_initialized = false;
  double _Ts = -1;
  // Runtime data
  int _rti_phase = 0;

};


}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_SOLVER_HPP_
