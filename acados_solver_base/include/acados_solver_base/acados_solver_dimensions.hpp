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

#ifndef ACADOS_SOLVER_BASE__ACADOS_SOLVER_DIMENSIONS_HPP_
#define ACADOS_SOLVER_BASE__ACADOS_SOLVER_DIMENSIONS_HPP_

/** @file
 *  @brief Definition of the acados::AcadosSolverDimensions class.
 */

namespace acados
{

class AcadosSolverDimensions
/**
 * @brief A container for the (fixed) dimensions of the imported Acados OCP.
 *
 */
{
public:
// Basic dimensions

  /// @brief Differential state vector size
  unsigned int nx; /*!< \f$n_x\f$ : size of the differential state vector \f$x \in \mathbb{R}^{n_x}\f$ */

  /// @brief Algebraic state vector size
  unsigned int nz; /*!< \f$n_z\f$ : size of the algebraic state vector \f$z\in \mathbb{R}^{n_z}\f$  */

  /// @brief Runtime parameters vector size
  unsigned int np; /*!< Number of runtime parameters */

  /// @brief Control vector size
  unsigned int nu; /*!< Size of the control vector \f$u\in \mathbb{R}^{n_u}\f$ */

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

}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_SOLVER_DIMENSIONS_HPP_
