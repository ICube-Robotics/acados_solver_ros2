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

bool utils::set_cost_Vx(AcadosSolver & solver, unsigned int stage, RowMajorXd & Vx)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_Vx()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;

  if (Vx.cols() != solver.nx()) {dimension_ok = false;}

  if (stage == 0) {
    if (Vx.rows() != solver.dims().ny_0) {dimension_ok = false;}
  } else if (stage == solver.N()) {
    if (Vx.rows() != solver.dims().ny_N) {dimension_ok = false;}
  } else {
    if (Vx.rows() != solver.dims().ny) {dimension_ok = false;}
  }

  if (!dimension_ok) {
    std::string err_msg = \
      "Error in 'Acados::utils::set_Vx()': Invalid matrix dimension!";
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

bool utils::set_cost_Vu(AcadosSolver & solver, unsigned int stage, RowMajorXd & Vu)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_Vu()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;

  if (Vu.cols() != solver.nu()) {dimension_ok = false;}

  if (stage == 0) {
    if (Vu.rows() != solver.dims().ny_0) {dimension_ok = false;}
  } else if (stage == solver.N()) {
    if (Vu.rows() != solver.dims().ny_N) {dimension_ok = false;}
  } else {
    if (Vu.rows() != solver.dims().ny) {dimension_ok = false;}
  }

  if (!dimension_ok) {
    std::string err_msg = \
      "Error in 'Acados::utils::set_Vu()': Invalid matrix dimension!";
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

bool utils::set_cost_Vz(AcadosSolver & solver, unsigned int stage, RowMajorXd & Vz)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_Vz()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;

  if (Vz.cols() != solver.nz()) {dimension_ok = false;}

  if (stage == 0) {
    if (Vz.rows() != solver.dims().ny_0) {dimension_ok = false;}
  } else if (stage == solver.N()) {
    if (Vz.rows() != solver.dims().ny_N) {dimension_ok = false;}
  } else {
    if (Vz.rows() != solver.dims().ny) {dimension_ok = false;}
  }

  if (!dimension_ok) {
    std::string err_msg = \
      "Error in 'Acados::utils::set_Vz()': Invalid matrix dimension!";
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

bool utils::set_cost_W(AcadosSolver & solver, unsigned int stage, RowMajorXd & W)
{
  if (stage > solver.N()) {
    std::string err_msg = "Error in 'Acados::utils::set_W()': Invalid stage request!";
    throw std::range_error(err_msg);
  }

  bool dimension_ok = true;
  if (stage == 0) {
    if (W.rows() != solver.dims().ny_0 || W.cols() != solver.dims().ny_0) {
      dimension_ok = false;
    }
  } else if (stage == solver.N()) {
    if (W.rows() != solver.dims().ny_N || W.cols() != solver.dims().ny_N) {
      dimension_ok = false;
    }
  } else {
    if (W.rows() != solver.dims().ny || W.cols() != solver.dims().ny) {
      dimension_ok = false;
    }
  }

  if (!dimension_ok) {
    std::string err_msg = \
      "Error in 'Acados::utils::set_W()': Invalid matrix dimension!";
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
  if (stage == 0) {
    if (y_ref.size() != solver.dims().ny_0) {dimension_ok = false;}
  } else if (stage == solver.N()) {
    if (y_ref.size() != solver.dims().ny_N) {dimension_ok = false;}
  } else {
    if (y_ref.size() != solver.dims().ny) {dimension_ok = false;}
  }

  if (!dimension_ok) {
    std::string err_msg = \
      "Error in 'Acados::utils::set_y_ref()': Invalid matrix dimension!";
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

}  // namespace acados
