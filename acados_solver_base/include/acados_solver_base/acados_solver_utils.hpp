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

#ifndef ACADOS_SOLVER_BASE__ACADOS_SOLVER_UTILS_HPP_
#define ACADOS_SOLVER_BASE__ACADOS_SOLVER_UTILS_HPP_

#include <Eigen/Dense>

#include <string>

// Acados C interface
#include "acados_c/ocp_nlp_interface.h"

#include "acados_solver_base/acados_solver.hpp"
#include "acados_solver_base/acados_types.hpp"


namespace acados
{

namespace utils
{

// ---------------------------------------------------------
// Convenience setters for commonly used cost variables
// ---------------------------------------------------------

/**
* @brief Generic setter for the cost.
*
* @note See  the definition of `ocp_nlp_cost_model_set` in
* acados/interfaces/acados_c/ocp_nlp_interface.c (https://github.com/acados)
* @note Available fields: 'Vx', 'Vu', 'Vz', 'yref', 'W', 'ext_cost_num_hess', 'zl', 'zu', 'Zl', 'Zu', etc.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N]
* @param field Name of the field
* @param value Matrix or vector containing the data (not modified)
* @return bool Status (true if all OK).
*/
template<typename Derived>
bool set_cost_field(
  AcadosSolver & solver,
  unsigned int stage,
  const std::string & field,
  Eigen::EigenBase<Derived> & value);

// Linear cost L = || Vx @ u + Vu @ u + Vz @ z - y_ref ||²_W
//
// Make sure the cost type is 'LINEAR_LS'
// ---------------------------------------------------------

/**
* @brief Set the Vx matrix used for 'LINEAR_LS' cost.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param Vx Matrix Vx
* @return bool Status (true if all OK).
*/
bool set_cost_Vx(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vx);

/**
* @brief Set the Vu matrix used for 'LINEAR_LS' cost.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param Vu Matrix Vu
* @return bool Status (true if all OK).
*/
bool set_cost_Vu(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vu);

/**
* @brief Set the Vz matrix used for 'LINEAR_LS' cost.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param Vz Matrix Vz
* @return bool Status (true if all OK).
*/
bool set_cost_Vz(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vz);

/**
* @brief Set the W cost matrix used for 'LINEAR_LS' cost.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param W Matrix W
* @return bool Status (true if all OK).
*/
bool set_cost_W(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & W);

/**
* @brief Set the y_ref vector for a given stage.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param y_ref Reference vector
* @return bool Status (true if all OK).
*/
bool set_cost_y_ref(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & y_ref);

// ------------------------------------------------------------
// Convenience setters for commonly used constraint variables
// ------------------------------------------------------------

/**
* @brief Generic setter for the constraints.
*
* @note See  the definition of `ocp_nlp_constraints_model_set` in
* acados/interfaces/acados_c/ocp_nlp_interface.c (https://github.com/acados)
* @note Available fields: 'lbx', 'ubx', 'lbu', 'ubu', 'lg', 'ug', 'lh', 'uh', 'uphi', 'C', 'D'
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N]
* @param field Name of the field
* @param value Matrix or vector containing the data (not modified, )
* @return bool Status (true if all OK).
*/
template<typename Derived>
bool set_constraint_field(
  AcadosSolver & solver,
  unsigned int stage,
  const std::string & field,
  Eigen::EigenBase<Derived> & value);

// Linear constraints
// ---------------------------------------------------------

// For basic utils, see `AcadosSolver::set_state_bounds` and `AcadosSolver::set_control_bounds`

// TODO(tpoignonec):
//     - Add resize_state_bounds(stage, idxbx, suppress_warnings = false)
//     - Add resize_state_bounds(stage, bx_index_map, suppress_warnings = false)
//     - Add resize_control_bounds(stage, idxbu, suppress_warnings = false)
//     - Add resize_control_bounds(stage, bu_index_map, suppress_warnings = false)

// Polytopic constraints of the form
// ---------------------------------------------------------
// General linear constraints of the form
//    \f$ D @ u+ C @ x \in [g_min; g_max] \f$
// except for terminal stage where
//    \f$ C @ x \in [g_min; g_max] \f$

/**
* @brief Set the C matrix used by polytopic constraints.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param C Matrix C
* @return bool Status (true if all OK).
*/
bool set_const_C(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & C);

/**
* @brief Set the D matrix used by polytopic constraints.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param D Matrix D
* @return bool Status (true if all OK).
*/
bool set_const_D(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & D);

/**
* @brief Set the lower bound g_min used by polytopic constraints.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param g_min Lower bounds on polytopic constraints.
* @return bool Status (true if all OK).
*/
bool set_const_g_min(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & g_min);

/**
* @brief Set the upper bound g_max used by polytopic constraints.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param g_max Upper bounds on polytopic constraints.
* @return bool Status (true if all OK).
*/
bool set_const_g_max(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & g_max);

// Ton-linear constraints
// ---------------------------------------------------------

/**
* @brief Set the lower bound h_min used by non-linear constraints.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param h_min Lower bounds on non-linear constraints.
* @return bool Status (true if all OK).
*/
bool set_const_h_min(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & h_min);

/**
* @brief Set the upper bound h_max used by non-linear constraints.
*
* @param solver Acados solver C++ wrapper handle
* @param stage Stage in [0;N].
* @param h_max Upper bounds on non-linear constraints.
* @return bool Status (true if all OK).
*/
bool set_const_h_max(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & h_max);

// ------------------------------------------------------------
// Convenience getters for solver stats
// ------------------------------------------------------------

/**
 * @brief Read a given field from the solver stats (solve must be called before...)
 *
 * @warning Dangerous method because the field type can be double or int!!!
 * Prefer the other get_stats_<field> methods when possible or check the documentation.
 *
 * @param solver Acados solver C++ wrapper handle
 * @param field Name of the field to retrieve (e.g., "time_tot", "cost_value", etc.)
 * @return double Value of the field
 */
template<typename T>
void unsafe_get_stats_field(AcadosSolver & solver, const std::string & field, T & value);

/**
 * @brief Retrieve the current cost value
 *
 * See `acados::utils::get_stats_field` for implementation details.
 *
 * @param solver Acados solver C++ wrapper handle
 * @return double cost value
 */
double get_stats_cost_value(AcadosSolver & solver);

/**
 * @brief Retrieve the SQP iteration at the last solve() call
 *
 * See `acados::utils::get_stats_field` for implementation details.
 *
 * @param solver Acados solver C++ wrapper handle
 * @return int SQP iteration
 */
int get_stats_sqp_iter(AcadosSolver & solver);

/**
 * @brief Retrieve the CPU time needed for the last solve() call
 *
 * See `acados::utils::get_stats_field` for implementation details.
 *
 * @param solver Acados solver C++ wrapper handle
 * @return double CPU time in seconds
 */
double get_stats_cpu_time(AcadosSolver & solver);

}  // namespace utils

}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_SOLVER_UTILS_HPP_
