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

#ifndef ACADOS_SOLVER_BASE__ACADOS_TYPES_HPP_
#define ACADOS_SOLVER_BASE__ACADOS_TYPES_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace acados
{
using IndexMap = std::unordered_map<std::string, std::vector<unsigned int>>;
using ValueMap = std::unordered_map<std::string, std::vector<double>>;
}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_TYPES_HPP_
