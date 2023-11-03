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
* @brief Set the Vx matrix used for 'LINEAR_LS' cost.
*
* @param stage Stage in [0;N].
* @param Vx Matrix Vx
* @return bool Status (true if all OK).
*/
bool set_cost_Vx(AcadosSolver & solver, unsigned int stage, RowMajorXd & Vx);

/**
* @brief Set the Vu matrix used for 'LINEAR_LS' cost.
*
* @param stage Stage in [0;N].
* @param Vu Matrix Vu
* @return bool Status (true if all OK).
*/
bool set_cost_Vu(AcadosSolver & solver, unsigned int stage, RowMajorXd & Vu);

/**
* @brief Set the Vz matrix used for 'LINEAR_LS' cost.
*
* @param stage Stage in [0;N].
* @param Vz Matrix Vz
* @return bool Status (true if all OK).
*/
bool set_cost_Vz(AcadosSolver & solver, unsigned int stage, RowMajorXd & Vz);

/**
* @brief Set the W cost matrix used for 'LINEAR_LS' cost.
*
* @param stage Stage in [0;N].
* @param W Matrix W
* @return bool Status (true if all OK).
*/
bool set_cost_W(AcadosSolver & solver, unsigned int stage, RowMajorXd & W);

/**
* @brief Set the y_ref vector for a given stage.
*
* @param stage Stage in [0;N].
* @param y_ref Reference vector
* @return bool Status (true if all OK).
*/
bool set_cost_y_ref(AcadosSolver & solver, unsigned int stage, Eigen::VectorXd & y_ref);


// ------------------------------------------------------------
// Convenience setters for commonly used constraint variables
// ------------------------------------------------------------

// TODO(tpoignonec): add h constr. setters and such

}  // namespace utils

}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_SOLVER_UTILS_HPP_
