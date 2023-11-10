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

namespace acados
{

bool utils::set_cost_Vx(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vx)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_Vx()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;

  if (Vx.cols() != solver.nx()) {dimension_ok = false;}

  int expected_rows = 0;
  if (stage == 0) {
    expected_rows = solver.dims().ny_0;
  } else if (stage == solver.N()) {
    expected_rows = solver.dims().ny_N;
  } else {
    expected_rows = solver.dims().ny;
  }
  if (Vx.rows() != expected_rows) {dimension_ok = false;}

  if (!dimension_ok) {
    std::string err_msg = \
      std::string("Error in 'Acados::utils::set_Vx()': Invalid matrix dimension!") + \
      "Hint: expected dimension are (" + \
      std::to_string(expected_rows) + ", " + \
      std::to_string(solver.nx()) + ").";
    throw std::invalid_argument(err_msg);
  }

  int ret = ocp_nlp_cost_model_set(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_in(),
    stage,
    "Vx",
    Vx.data()
  );
  return ret == 0;
}

bool utils::set_cost_Vu(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vu)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_Vu()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;

  if (Vu.cols() != solver.nu()) {dimension_ok = false;}

  int expected_rows = 0;
  if (stage == 0) {
    expected_rows = solver.dims().ny_0;
  } else if (stage == solver.N()) {
    expected_rows = solver.dims().ny_N;
  } else {
    expected_rows = solver.dims().ny;
  }
  if (Vu.rows() != expected_rows) {dimension_ok = false;}

  if (!dimension_ok) {
    std::string err_msg = \
      std::string("Error in 'Acados::utils::set_Vu()': Invalid matrix dimension!") + \
      "Hint: expected dimension are (" + \
      std::to_string(expected_rows) + ", " + \
      std::to_string(solver.nu()) + ").";
    throw std::invalid_argument(err_msg);
  }

  int ret = ocp_nlp_cost_model_set(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_in(),
    stage,
    "Vu",
    Vu.data()
  );
  return ret == 0;
}

bool utils::set_cost_Vz(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & Vz)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_Vz()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;
  int expected_rows = 0;
  if (stage == 0) {
    expected_rows = solver.dims().ny_0;
  } else if (stage == solver.N()) {
    expected_rows = solver.dims().ny_N;
  } else {
    expected_rows = solver.dims().ny;
  }
  if (Vz.rows() != expected_rows) {dimension_ok = false;}

  if (!dimension_ok) {
    std::string err_msg = \
      std::string("Error in 'Acados::utils::set_Vz()': Invalid matrix dimension!") + \
      "Hint: expected dimension are (" + \
      std::to_string(expected_rows) + ", " + \
      std::to_string(solver.nz()) + ").";
    throw std::invalid_argument(err_msg);
  }

  int ret = ocp_nlp_cost_model_set(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_in(),
    stage,
    "Vz",
    Vz.data()
  );
  return ret == 0;
}

bool utils::set_cost_W(AcadosSolver & solver, unsigned int stage, Eigen::MatrixXd & W)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_W()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;
  int expected_size = 0;
  if (stage == 0) {
    expected_size = solver.dims().ny_0;
  } else if (stage == solver.N()) {
    expected_size = solver.dims().ny_N;
  } else {
    expected_size = solver.dims().ny;
  }
  if ((W.rows() != expected_size) || (W.cols() != expected_size)) {
    dimension_ok = false;
  }

  if (!dimension_ok) {
    std::string err_msg = \
      std::string("Error in 'Acados::utils::set_W()': Invalid matrix dimension!") + \
      "Hint: expected dimension are (" + \
      std::to_string(expected_size) + ", " + \
      std::to_string(expected_size) + ").";
    throw std::invalid_argument(err_msg);
  }

  int ret = ocp_nlp_cost_model_set(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_in(),
    stage,
    "W",
    W.data()
  );
  return ret == 0;
}

bool utils::set_cost_y_ref(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & y_ref)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_y_ref()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;
  int expected_size = 0;
  if (stage == 0) {
    expected_size = solver.dims().ny_0;
  } else if (stage == solver.N()) {
    expected_size = solver.dims().ny_N;
  } else {
    expected_size = solver.dims().ny;
  }
  if (y_ref.size() != expected_size) {dimension_ok = false;}

  if (!dimension_ok) {
    std::string err_msg = \
      std::string("Error in 'Acados::utils::set_y_ref()': Invalid matrix dimension!") + \
      "Hint: expected vector length : " + std::to_string(expected_size) + ".";
    throw std::invalid_argument(err_msg);
  }

  int ret = ocp_nlp_cost_model_set(
    solver.get_nlp_config(),
    solver.get_nlp_dims(),
    solver.get_nlp_in(),
    stage,
    "y_ref",
    y_ref.data()
  );
  return ret == 0;
}

// ------------------------------------------------------------
// Convenience setters for commonly used constraint variables
// ------------------------------------------------------------

// TODO(tpoignonec): add h constr. setters and such


// ------------------------------------------------------------
// Convenience getters for solver stats
// ------------------------------------------------------------

double utils::get_stats_cost_value(AcadosSolver & solver)
{
  double cost_value = 0.0;
  ocp_nlp_get(
    solver.get_nlp_config(),
    solver.get_nlp_solver(),
    "cost_value",
    &cost_value
  );
  return cost_value;
}

double utils::get_stats_sqp_iter(AcadosSolver & solver)
{
  double sqp_iter = 0.0;
  ocp_nlp_get(
    solver.get_nlp_config(),
    solver.get_nlp_solver(),
    "sqp_iter",
    &sqp_iter
  );
  return sqp_iter;
}

double utils::get_stats_cpu_time(AcadosSolver & solver)
{
  double cpu_time = 0.0;
  ocp_nlp_get(
    solver.get_nlp_config(),
    solver.get_nlp_solver(),
    "time_tot",
    &cpu_time
  );
  return cpu_time;
}

}  // namespace acados
