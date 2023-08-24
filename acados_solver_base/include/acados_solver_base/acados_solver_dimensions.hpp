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

namespace acados
{

class AcadosSolverDimensions
{
public:
// Basic dimensions
  int nx = -1;
  int nz = -1;
  int np = -1;
  int nu = -1;
// Bounds on x and u
  int nbx = -1;
  int nbx0 = -1;
  int nbxN = -1;
  int nbu = -1;
// Polytope constraints
  int ng = -1;
  int ngN = -1;
// Non-linear constraints
  int nh = -1;
  int nhN = -1;
  int nphi = -1;
  int nphiN = -1;
// Slack
  int ns = -1;
  int nsN = -1;
  int nsbx = -1;
  int nsbxN = -1;
  int nsbu = -1;
  int nsh = -1;
  int nshN = -1;
  int nsg = -1;
  int nsgN = -1;
  int nsphi = -1;
  int nsphiN = -1;
// Cost ref.
  int ny = -1;
  int ny0 = -1;
  int nyN = -1;
// Misc.
  int nr = -1;
};

}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_SOLVER_DIMENSIONS_HPP_
