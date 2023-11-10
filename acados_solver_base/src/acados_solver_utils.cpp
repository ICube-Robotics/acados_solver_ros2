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

#include "acados_solver_base/acados_solver_utils.hpp"
#include <numeric>  // for std::iota
#include <stdexcept>
#include <iostream>

namespace acados
{


// ---------------------------------------------------------
// Convenience setters for commonly used cost variables
// ---------------------------------------------------------

template<typename Derived>
bool utils::set_cost_field(
  AcadosSolver & solver,
  unsigned int stage,
  const std::string & field,
  Eigen::EigenBase<Derived> & value)
{
  if (stage > solver.N()) {
    throw std::range_error(
            "Acados::utils::set_cost_field could not set '" + \
            field + "'! Invalid stage request.");
  }

  std::vector dim_field = {0, 0};
  ocp_nlp_cost_dims_get_from_attr(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_out(),
    stage,
    field.c_str(),
    dim_field.data()
  );

  bool valid_dimensions = true;
  if (dim_field[1] == 0) {
    // Vector: test length
    valid_dimensions = (value.size() == dim_field[0]);
  } else {
    // Matrix: test both dimensions
    valid_dimensions = ((value.rows() == dim_field[0]) && (value.cols() == dim_field[1]));
  }

  if (!valid_dimensions) {
    std::ostringstream stringStream_msg;
    stringStream_msg << "" \
                     << "Acados::utils::set_cost_field could not set ' " << field << "'!" \
                     << "Invalid dimensions: expected (" \
                     << dim_field[0] << ", " << dim_field[1] << "), but got (" \
                     << value.rows() << ", " << value.cols() << ")." << std::endl;
    std::string error_msg = stringStream_msg.str();
    throw std::runtime_error(error_msg);
  }

  int ret = ocp_nlp_cost_model_set(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_in(),
    stage,
    field.c_str(),
    value.derived().data()
  );
  return ret == 0;
}


// Linear cost
// ---------------------------------------------------------

bool utils::set_cost_Vx(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vx)
{
  return set_cost_field(solver, stage, "Vx", Vx);
}

bool utils::set_cost_Vu(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vu)
{
  return set_cost_field(solver, stage, "Vu", Vu);
}

bool utils::set_cost_Vz(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vz)
{
  return set_cost_field(solver, stage, "Vz", Vz);
}

bool utils::set_cost_W(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & W)
{
  return set_cost_field(solver, stage, "W", W);
}

bool utils::set_cost_y_ref(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & y_ref)
{
  return set_cost_field(solver, stage, "y_ref", y_ref);
}

// Non-linear linear cost
// ---------------------------------------------------------


// ------------------------------------------------------------
// Convenience setters for commonly used constraint variables
// ------------------------------------------------------------

template<typename Derived>
bool utils::set_constraint_field(
  AcadosSolver & solver,
  unsigned int stage,
  const std::string & field,
  Eigen::EigenBase<Derived> & value)
{
  if (stage > solver.N()) {
    throw std::range_error(
            "Acados::utils::set_constraint_field could not set '" + \
            field + "'! Invalid stage request.");
  }

  std::vector dim_field = {0, 0};
  ocp_nlp_constraint_dims_get_from_attr(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_out(),
    stage,
    field.c_str(),
    dim_field.data()
  );

  bool valid_dimensions = true;
  if (dim_field[1] == 0) {
    // Vector: test length
    valid_dimensions = (value.size() == dim_field[0]);
  } else {
    // Matrix: test both dimensions
    valid_dimensions = ((value.rows() == dim_field[0]) && (value.cols() == dim_field[1]));
  }

  if (!valid_dimensions) {
    std::ostringstream stringStream_msg;
    stringStream_msg << "" \
                     << "Acados::utils::set_constraint_field could not set ' " << field << "'!" \
                     << "Invalid dimensions: expected (" \
                     << dim_field[0] << ", " << dim_field[1] << "), but got (" \
                     << value.rows() << ", " << value.cols() << ")." << std::endl;
    std::string error_msg = stringStream_msg.str();
    throw std::runtime_error(error_msg);
  }

  int ret = ocp_nlp_constraints_model_set(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_in(),
    stage,
    field.c_str(),
    value.derived().data()
  );
  return ret == 0;
}

// Linear constraints
// ---------------------------------------------------------

// For basic utils, see `AcadosSolver::set_state_bounds` and `AcadosSolver::set_control_bounds`

// General linear constraints
// ---------------------------------------------------------

bool utils::set_const_C(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & C)
{
  return set_constraint_field(solver, stage, "C", C);
}

bool utils::set_const_D(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & D)
{
  return set_constraint_field(solver, stage, "D", D);
}

bool utils::set_const_g_min(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & g_min)
{
  return set_constraint_field(solver, stage, "lg", g_min);
}

bool utils::set_const_g_max(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & g_max)
{
  return set_constraint_field(solver, stage, "ug", g_max);
}


// Non-linear constraints
// ---------------------------------------------------------

bool utils::set_const_h_min(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & h_min)
{
  return set_constraint_field(solver, stage, "lh", h_min);
}


bool utils::set_const_h_max(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & h_max)
{
  return set_constraint_field(solver, stage, "uh", h_max);
}


// ------------------------------------------------------------
// Convenience getters for solver stats
// ------------------------------------------------------------

template<typename T>
void utils::unsafe_get_stats_field(AcadosSolver & solver, const std::string & field, T & value)
{
  ocp_nlp_get(
    solver.get_nlp_config(),
    solver.get_nlp_solver(),
    field.c_str(),
    &value
  );
}

double utils::get_stats_cost_value(AcadosSolver & solver)
{
  double cost_value = 0.0;
  unsafe_get_stats_field(solver, "cost_value", cost_value);
  return cost_value;
}

int utils::get_stats_sqp_iter(AcadosSolver & solver)
{
  int sqp_iter = 0.0;
  unsafe_get_stats_field(solver, "sqp_iter", sqp_iter);
  return sqp_iter;
}

double utils::get_stats_cpu_time(AcadosSolver & solver)
{
  double time_tot = 0.0;
  unsafe_get_stats_field(solver, "time_tot", time_tot);
  return time_tot;
}

}  // namespace acados
