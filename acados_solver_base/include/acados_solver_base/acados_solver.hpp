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
    /// @note Typically equal to nx, used to set initial state before calling `solve()`.
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

    /// @brief Dimensions of the image of the inner nonlinear function in positive definite constr.
    /// See Acados documentation.
    unsigned int nr;
  };

public:
// Acados solver public API

  /**
   * @brief Constructor of the AcadosSolver object
   *
   * Does nothing for now.
   *
   * @warning Any class inheriting from AcadosSolver will have to correctly initialize the `AcadosSolver::_dims` object.
   *
   */
  AcadosSolver();

  /**
   * @brief Destructor of the AcadosSolver object
   *
   * @warning The `free_memory()` function must called by the destructor of any class inheriting from AcadosSolver.
   *          Otherwise, the allocated memory won't be fully released.
   */
  virtual ~AcadosSolver();

// Solver management

  /**
   * @brief Initialize the solver and set the (constant) sampling intervals.
   *
   * @todo Add alternative `init()` function for non-constant sampling intervals.
   *
   * @param N Number of shooting nodes (strictly positive).
   * @param Ts Sampling time in seconds.
   * @return int (zero if all OK).
   */
  int init(unsigned int N, double Ts);

  /**
   * @brief Resets the Acados solver, including the internal QP solver and the RTI phase cache.
   *
   * @return int (zero if all OK).
   */
  int reset();

// Solve and introspection

  /**
   * @brief Solve the non-linear optimization problem given the provided initial state, constraints, etc.
   *
   * The method returns the internal Acados status.
   *
   * @return 0 : All OK.
   * @return 1 : Invalid input (NaN detected).
   * @return 2 : Maximum number of iteration reached.
   * @return 3 : Minimum step length reached, could possibly be a NaN solution too (if using HPIPM)
   * @return 4 : QP solver failed.
   * @return other : Not OK.
   */
  int solve();

// Setters

  /**
   * @brief Set the initial state values (i.e., add constraints on initial state)
   *
   * @param x_0 State values at initial stage.
   * @return int (zero if all OK)
   */
  int set_initial_state_values(ValueVector & x_0);

  /**
   * @brief Set the initial state values (i.e., add constraints on initial state) from a map of key/ValueVector pairs.
   *
   * @param x_0_map ValueMap of the initial state values.
   * @return int (zero if all OK)
   */
  int set_initial_state_values(ValueMap const & x_0_map);

  /**
   * @brief Set (differential) state bounds at a given stage.
   *
   * @param stage Stage in [0;N].
   * @param idxbx Indexes of the bounded state variables.
   * @param lbx Vector of lower bounds.
   * @param ubx Vector of uower bounds.
   * @return int (zero if all OK)
   */
  int set_state_bounds(
    unsigned int stage,
    IndexVector & idxbx,
    ValueVector & lbx,
    ValueVector & ubx);

  /**
   * @brief Set (differential) state bounds for all stages.
   *
   * The bounds are therefore equal at each stage.
   *
   * @param idxbx Indexes of the bounded state variables.
   * @param lbx Vector of lower bounds.
   * @param ubx Vector of uower bounds.
   * @return int (zero if all OK)
   */
  int set_state_bounds(
    IndexVector & idxbx,
    ValueVector & lbx,
    ValueVector & ubx);

  /**
   * @brief Set control bounds at a given stage.
   *
   * @param stage Stage in [0;N].
   * @param idxbu Indexes of the bounded control variables.
   * @param lbu Vector of lower bounds.
   * @param ubu Vector of uower bounds.
   * @return int (zero if all OK)
   */
  int set_control_bounds(
    unsigned int stage,
    IndexVector & idxbu,
    ValueVector & lbu,
    ValueVector & ubu);
  /**
   * @brief Set control bounds for all stages.
   *
   * The bounds are therefore equal at each stage.
   *
   * @param idxbu Indexes of the bounded control variables.
   * @param lbu Vector of lower bounds.
   * @param ubu Vector of uower bounds.
   * @return int (zero if all OK)
   */
  int set_control_bounds(
    IndexVector & idxbu,
    ValueVector & lbu,
    ValueVector & ubu);

// Runtime parameters

  /**
   * @brief Set the runtime parameters for one stage from ordered value vector.
   *
   * @param stage Stage in [0;N].
   * @param p_i Vector containing the (ordered) runtime parameters.
   * @return int Status (zero if all OK).
   */
  int set_runtime_parameters(unsigned int stage, ValueVector & p_i);

  /**
   * @brief Set the runtime parameters for one stage from a key/values map.
   *
   * @note If the parameters are the same for all stages, prefer the `set_runtime_parameters(ValueMap const &)` alternative.
   *
   * @param stage Stage in [0;N].
   * @param p_i_map Key to ValueVector map containing the runtime parameters.
   * @return int Status (zero if all OK).
   */
  int set_runtime_parameters(unsigned int stage, ValueMap const & p_i_map);


  /**
   * @brief Set the runtime parameters for all stages at once from ordered value vector.
   *
   * The runtime parameters for ALL stages are equal.
   *
   * @param p_i Vector containing the (ordered) runtime parameters.
   * @return int Status (zero if all OK).
   */
  int set_runtime_parameters(ValueVector & p_i);

  /**
   * @brief Set the runtime parameters for all stages at once from a key/values map.
   *
   * The runtime parameters for ALL stages are equal.
   * Once the ordered vector of parameters is extracted, it is the `set_runtime_parameters(unsigned int, ValueVector &)` function that is used.
   *
   * @param p_i_map Key to ValueVector map containing the runtime parameters.
   * @return int Status (zero if all OK).
   */
  int set_runtime_parameters(ValueMap const & p_i_map);

// Initialization state

  /**
   * @brief Initialize the (differential) state values for a stage from an ordered value vector.
   *
   * @warning This function (and other "initialization" functions similar to this one) only initialize the solver state/control values.
   * These function should only be used before a `AcadoSolver::solve()` call!
   * Also, initializing the state values for stage zero is not equivalent to set the initial state (see `AcadoSolver::set_initial_state_values()`).
   *
   * @param stage Stage in [0;N].
   * @param x_i Vector containing the (ordered) state values.
   * @return int Status (zero if all OK).
   */
  int initialize_state_values(unsigned int stage, ValueVector & x_i);

  /**
   * @brief Initialize the (differential) state values for a stage from a key/values map.
   *
   * @note If the parameters are the same for all stages, prefer the `set_runtime_parameters(ValueMap const &)` alternative.
   *
   * @param stage Stage in [0;N].
   * @param x_i_map Key to ValueVector map containing the initial state values.
   * @return int Status (zero if all OK).
   */
  int initialize_state_values(unsigned int stage, ValueMap const & x_i_map);

  /**
    * @brief Initialize the (differential) state values for ALL stages at once from an ordered value vector.
    *
    * @param x_i Vector containing the (ordered) state values.
    * @return int Status (zero if all OK).
    */
  int initialize_state_values(ValueVector & x_i);

  /**
   * @brief Initialize the (differential) state values for ALL stages at once from a key/values map.
   *
   * @param x_i_map Key to ValueVector map containing the initial state values.
   * @return int Status (zero if all OK).
   */
  int initialize_state_values(ValueMap const & x_i_map);

// Initialization controls

  /**
   * @brief Initialize the control variable values for one stage from an ordered value vector.
   *
   * @param stage Stage in [0;N].
   * @param u_i Vector containing the (ordered) control variables values.
   * @return int Status (zero if all OK).
   */
  int initialize_control_values(unsigned int stage, ValueVector & u_i);

  /**
   * @brief Initialize the control variable values for one stage from a key/values map.
   *
   * @param stage Stage in [0;N].
   * @param u_i_map Key to ValueVector map containing the control variables values.
   * @return int Status (zero if all OK).
   */
  int initialize_control_values(unsigned int stage, ValueMap const & u_i_map);

  /**
   * @brief Initialize the control variable values for ALL stages at once from an ordered value vector.
   *
   * @param u_i Vector containing the (ordered) control variables values.
   * @return int Status (zero if all OK).
   */
  int initialize_control_values(ValueVector & u_i);

  /**
   * @brief Initialize the control variable values for ALL stages at once from a key/values map.
   *
   * @param u_i_map Key to ValueVector map containing the control variables values.
   * @return int Status (zero if all OK).
   */
  int initialize_control_values(ValueMap const & u_i_map);

// Getters
  /**
   * @brief Retrieve the differential state variables at a given stage.
   *
   * @param stage Stage in [0;N].
   * @return ValueVector Vector of (ordered) state values.
   */
  ValueVector get_state_values(unsigned int stage);

  /**
   * @brief Retrieve the differential state variables at a given stage and package them as a ValueMap.
   *
   * @param stage Stage in [0;N].
   * @return ValueMap Key/ValueVector map containing of differential state values.
   */
  ValueMap get_state_values_as_map(unsigned int stage);

  /**
   * @brief Retrieve the algebraic state variables at a given stage.
   *
   * @param stage Stage in [0;N].
   * @return ValueVector Vector of (ordered) state values.
   */
  ValueVector get_algebraic_state_values(unsigned int stage);


  /**
   * @brief Retrieve the algebraic state variables at a given stage and package them as a ValueMap.
   *
   * @param stage Stage in [0;N].
   * @return ValueMap Key/ValueVector map containing the algebraic state values.
   */
  ValueMap get_algebraic_state_values_as_map(unsigned int stage);

  /**
   * @brief Retrieve the control variables at a given stage.
   *
   * @param stage Stage in [0;N].
   * @return ValueVector Vector of (ordered) control variable values.
   */
  ValueVector get_control_values(unsigned int stage);

  /**
   * @brief Retrieve the control variables at a given stage and package them as a ValueMap.
   *
   * @param stage Stage in [0;N].
   * @return ValueMap Key/ValueVector map containing the control variable values.
   */
  ValueMap get_control_values_as_map(unsigned int stage);

// Getters variable mappings

  /**
   * @brief Returns a constant reference to the Key/IndexVector map of differential state variables internally stored in `AcadosSolver::_x_index_map`.
   *
   * @return const IndexMap&
   */
  const IndexMap & x_index_map() const;

  /**
   * @brief Returns a constant reference to the Key/IndexVector map of algebraic state variables internally stored in `AcadosSolver::_z_index_map`.
   *
   * @return const IndexMap&
   */
  const IndexMap & z_index_map() const;

  /**
   * @brief Returns a constant reference to the Key/IndexVector map of runtime parameters internally stored in `AcadosSolver::_p_index_map`.
   *
   * @return const IndexMap&
   */
  const IndexMap & p_index_map() const;

  /**
   * @brief Returns a constant reference to the Key/IndexVector map of control variables internally stored in `AcadosSolver::_u_index_map`.
   *
   * @return const IndexMap&
   */
  const IndexMap & u_index_map() const;

// Values map utils

  /**
   * @brief Check if the total number of indexes contained in the provided index map is equal to an expected number.
   *
   * @param index_map The Key/IndexVector map to be tested for size consistency.
   * @param expected_total_indexes The expected number of indexes.
   * @return true if the size is consistent
   * @return false otherwise
   */
  static bool is_map_size_consistent(
    IndexMap const & index_map,
    unsigned int expected_total_indexes);

  /**
   * @brief Check if the total number of values contained in the provided value map is equal to an expected number.
   *
   * @param values_map The Key/ValueVector map to be tested for size consistency.
   * @param expected_total_indexes The expected number of values.
   * @return true if the size is consistent
   * @return false otherwise
   */
  static bool is_map_size_consistent(
    ValueMap const & values_map,
    unsigned int expected_total_indexes);

  /**
   * @brief Check if for each key-index pair contained in the index map there is a value provided by the value map.
   *
   * It is considered complete even if there are too many values (e.g., unused keys in values_map).
   *
   * @param index_map The Key/IndexVector map to be tested for size consistency.
   * @param values_map The Key/ValueVector map to be tested for size consistency.
   * @return true if the value map is complete.
   * @return false otherwise.
   */
  static bool is_values_map_complete(IndexMap const & index_map, ValueMap const & values_map);

  /**
   * @brief Fill (and resize if necessary) a vector with the values contained in the value map.
   *
   * The index map is used to arrange the values within the vector.
   *
   * @throws std::invalid_argument
   *
   * @todo Overloaded function without the `vector_size` argument.
     The `vector_size` argument could (should?) be optional since it can be retrieved from the number of indexes contained in `index_map`.
   *
   * @param index_map Mapping between keys and indexes.
   * @param values_map Mapping between keys and values to write in the value vector.
   * @param vector_size The expected size of the resulting value vector.
   * @param values The output container where the values will be written.
   */
  static void fill_vector_from_map(
    IndexMap const & index_map,
    ValueMap const & values_map,
    unsigned int vector_size,
    ValueVector & values);

  /**
   * @brief Returns a vector containing the values provided through the value map.
   *
   * @param index_map Mapping between keys and indexes.
   * @param values_map Mapping between keys and values to write in the value vector.
   * @param vector_size The expected size of the resulting value vector.
   * @return ValueVector The resulting value vector.
   */
  static ValueVector value_vector_from_map(
    IndexMap const & index_map,
    ValueMap const & values_map,
    unsigned int vector_size);

  /**
    * @brief Returns a vector ValueMap object containing the values of the value vector.
    *
    * @warning The `values` must have been encoded as described by the `index_map` argument.
    *
    * @param index_map Mapping between keys and indexes.
    * @param values Vector of (ordered) values.
    */
  static ValueMap create_map_from_values(
    IndexMap const & index_map,
    ValueVector const & values);

// Problem dimensions and convenience getters for commonly used attributes

  /**
   * @brief Returns the (fixed) dimensions of the OCP problem stored in `AcadosSolver::_dims`.
   *
   * @return const Dimensions& The dimensions
   */
  const Dimensions & dims() const;

  /**
   * @brief Returns the dimensions of the differential state vector.
   *
   * This is a convenience wrapper of the call `dims().nx` (see `dims()`).
   *
   * @return unsigned int nx
   */
  unsigned int  nx() const;

  /**
   * @brief Returns the dimensions of the algebraic state vector.
   *
   * This is a convenience wrapper of the call `dims().nz` (see `dims()`).
   *
   * @return unsigned int nz
   */
  unsigned int  nz() const;

  /**
   * @brief Returns the number of runtime parameters.
   *
   * This is a convenience wrapper of the call `dims().np` (see `dims()`).
   *
   * @return unsigned int np
   */
  unsigned int  np() const;

  /**
   * @brief Returns the number of control variables.
   *
   * This is a convenience wrapper of the call `dims().nu` (see `dims()`).
   *
   * @return unsigned int nu
   */
  unsigned int  nu() const;

  /**
   * @brief Returns the number of shooting nodes.
   *
   * @return unsigned int N
   */
  unsigned int  N() const;

  /**
   * @brief Returns the (constant) sampling interval Ts (in seconds).
   *
   * If the sampling intervals are of variable duration (see `sampling_intervals()`), the function returns `-1`.
   *
   * @return unsigned int Ts
   */
  double Ts() const;


  /**
   * @brief Returns the vector of sampling interval (in seconds).
   *
   * @return std::vector<double> the vector of sampling intervals
   */
  std::vector<double> sampling_intervals() const;

protected:
  /// @brief Fixed dimensions of the imported Acados OCP.
  Dimensions _dims;

  /// @brief Unordered map used to set or retrieve the values of diff. state variables by name.
  IndexMap _x_index_map;

  /// @brief Unordered map used to set or retrieve the values of algebraic state variables by name.
  IndexMap _z_index_map;

  /// @brief Unordered map used to set the values of runtime parameters by name.
  IndexMap _p_index_map;

  /// @brief Unordered map used to set or retrieve the values of control variables by name.
  IndexMap _u_index_map;

// Solver-specific functions

  /// @brief Reinitialize the maps used to set or retrieve value using string variable names.
  virtual int create_index_maps() = 0;

protected:
  /**
   * @brief Free all the allocated memory.
   *
   * Triggers calls to `internal_free()`, `internal_free_capsule()`, etc.
   *
   * @warning Should be called by the destructor of any class inheriting from AcadosSolver.
   *
   * @return int (zero if all OK).
   */
  int free_memory();

// Imported Acados solver C-code interface ("internal" --> internal use only!)

  /**
   * @brief Create the Acados solver's capsule used by the Acados C-interface.
   *
   * Wrapper of the `<acados_model_name>_acados_create_capsule()` C function.
   * Note that the storage of the capsule itself is handled by the inherited class.
   *
   * @return int (zero if all OK).
   */
  virtual int internal_create_capsule() = 0;

  /**
   * @brief Initialize Acados solver and (optionally) redefine the shooting nodes.
   *
   * This is a wrapper of the `<acados_model_name>_acados_create_with_discretization(...)` C function.
   * For constant sampling intervals, provide `new_time_steps = nullptr` as argument.
   *
   * @param n_time_steps number of shooting nodes.
   * @param new_time_steps C-array of sampling intervals (in seconds).
   * @return int (zero if all OK).
   */
  virtual int internal_create_with_discretization(int n_time_steps, double * new_time_steps) = 0;

  /**
   * @brief Reset the solver memory and (opt.) the internal QP solver.
   *
   * This is a wrapper of the `<acados_model_name>_acados_reset(...)` C function.
   *
   * @param reset_qp_solver_mem Reset if `=1`.
   * @return int (zero if all OK).
   */
  virtual int internal_reset(int reset_qp_solver_mem = 1) = 0;

  /**
   * @brief Free the memory allocated for the solver.
   *
   * This is a wrapper of the `<acados_model_name>_acados_free(...)` C function.
   * Call `internal_free()` before `internal_free_capsule()`.
   *
   * @return int (zero if all OK).
   */
  virtual int internal_free() = 0;

  /**
   * @brief Free the memory allocated for the capsule.
   *
   * This is a wrapper of the `<acados_model_name>_acados_free_capsule(...)` C function.
   *
   * @return int (zero if all OK).
   */
  virtual int internal_free_capsule() = 0;

  /**
   * @brief Update the QP solver `qp_solver_cond_N` setting.
   *
   * This is a wrapper of the `<acados_model_name>_acados_update_qp_solver_cond_N(...)` C function.
   *
   * @param qp_solver_cond_N New horizon after partial condensing.
   * @return int (zero if all OK).
   */
  virtual int internal_update_qp_solver_cond_N(int qp_solver_cond_N) = 0;

  /**
  * @brief Internal update of the solver runtime parameters.
  *
  * This is a wrapper of the `<acados_model_name>_acados_update_params(...)` C function.
  *
  * @param stage Stage in [0;N].
  * @param value Parameter values (C-array).
  * @param np Number of parameters.
  * @return int (zero if all OK).
  */
  virtual int internal_update_params(unsigned int stage, double * value, int np) = 0;

  /**
  * @brief A sparse implementation of `internal_update_params_sparse()`.
  *
  * This is a wrapper of the `<acados_model_name>_acados_update_params_sparse(...)` C function.
  *
  * @param stage Stage in [0;N].
  * @param idx Parameter indexes (C-array).
  * @param p Parameter values (C-array).
  * @param n_update Number of parameters to update.
  * @return int (zero if all OK).
  */
  virtual int internal_update_params_sparse(
    unsigned int stage,
    int * idx,
    double * p,
    int n_update) = 0;

  /**
  * @brief Solve the non-linear optimization`.
  *
  * This is a wrapper of the `<acados_model_name>_acados_solve(...)` C function.
  *
  * @return int (zero if all OK). See Acados documentation for a list of return flags.
  */
  virtual int internal_solve() = 0;

  /**
  * @brief Print the solver stats (e.g., number of SQP iters, number of QP iters, etc.) to the console.
  *
  * This is a wrapper of the `<acados_model_name>_acados_print_stats(...)` C function.
  */
  virtual void internal_print_stats() const = 0;

public:
  /**
  * @brief Legacy C-function to retrieve the C-interface `ocp_nlp_in` property of the solver.
  *
  * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_nlp_in(...)` C function.
  *
  * @return ocp_nlp_in* Pointer towards the `ocp_nlp_in` object.
  */
  virtual ocp_nlp_in * get_nlp_in() const = 0;

  /**
  * @brief Legacy C-function to retrieve the C-interface `ocp_nlp_out` property of the solver.
  *
  * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_nlp_out(...)` C function.
  *
  * @return ocp_nlp_out* Pointer towards the `ocp_nlp_out` object.
  */
  virtual ocp_nlp_out * get_nlp_out() const = 0;

  /**
  * @brief Legacy C-function to retrieve the C-interface `ocp_nlp_out` property of the solver.
  *
  * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_sens_out(...)` C function.
  * The function is used to retrieve the sensibility matrices.
  *
  * @return ocp_nlp_out* Pointer towards the `ocp_nlp_out` object.
  */
  virtual ocp_nlp_out * get_sens_out() const = 0;

  /**
   * @brief Legacy C-function to retrieve the C-interface `ocp_nlp_solver` property of the solver.
   *
   * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_nlp_solver(...)` C function.
   *
   * @return ocp_nlp_solver* Pointer towards the `ocp_nlp_solver` object.
   */
  virtual ocp_nlp_solver * get_nlp_solver() const = 0;

  /**
   * @brief Legacy C-function to retrieve the C-interface `ocp_nlp_config` property of the solver.
   *
   * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_nlp_config(...)` C function.
   *
   * @return ocp_nlp_config* Pointer towards the `ocp_nlp_config` object.
   */
  virtual ocp_nlp_config * get_nlp_config() const = 0;

  /**
   * @brief Legacy C-function to retrieve the C-interface `get_nlp_opts` property of the solver.
   *
   * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_nlp_opts(...)` C function.
   *
   * @return void* Pointer towards the `nlp_opts` object.
   */
  virtual void * get_nlp_opts() const = 0;

  /**
   * @brief Legacy C-function to retrieve the C-interface `ocp_nlp_dims` property of the solver that contains the dimensions of the problem.
   *
   * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_nlp_dims(...)` C function.
   *
   * @return ocp_nlp_dims* Pointer towards the `ocp_nlp_dims` object.
   */
  virtual ocp_nlp_dims * get_nlp_dims() const = 0;

  /**
   * @brief Legacy C-function to retrieve the C-interface `ocp_nlp_plan` property of the solver.
   *
   * This is a wrapper of the auto-generated `<acados_model_name>_acados_get_nlp_plan(...)` C function.
   *
   * @return ocp_nlp_plan_t* Pointer towards the `ocp_nlp_plan` object.
   */
  virtual ocp_nlp_plan_t * get_nlp_plan() const = 0;

  /**
   * @brief Legacy C-function to retrieve the number of runtime parameters from the Acados capsule.
   *
   * This function returns the field `capsule->nlp_np` where `capsule` is the internal pointer to the acados solver data.
   *
   * @return unsigned int np
   */
  virtual unsigned int get_nlp_np() const = 0;

// Imported Acados SIM solver C-code interface (for internal use only!)

protected:
  // TODO(tpoignonec) : add sim solver private c-interface

public:
  // TODO(tpoignonec) : add sim solver public c-interface

  // Private own attributes

private:
  /// @brief Internal flag set to true when `init()` is called.
  bool _is_initialized = false;

  /// @brief Sampling time in seconds. If the sampling intervals are variable, then `Ts = -1`.
  double _Ts = -1;
// Runtime data

  /// @brief Internal variable used to store the SQP RTI phase.
  int _rti_phase = 0;
};

}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_SOLVER_HPP_
