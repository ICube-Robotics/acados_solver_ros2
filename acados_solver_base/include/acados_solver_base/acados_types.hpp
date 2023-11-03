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

#include <Eigen/Dense>

#include <string>
#include <unordered_map>
#include <vector>

namespace acados
{

/// @brief List of indexes.
using IndexVector = std::vector<unsigned int>;

/// @brief List of values (double).
using ValueVector = std::vector<double>;

/// @brief Mapping between keys (`std::string`) and indexes (`acados::IndexVector`).
using IndexMap = std::unordered_map<std::string, IndexVector>;

/// @brief Mapping between keys (`std::string`) and data (`acados::ValueVector`).
using ValueMap = std::unordered_map<std::string, ValueVector>;

/// @brief Dynamic size row-major array (hence compatible with Acados C-arrays).
using RowMajorXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/// @brief Dynamic size colum-major array (by default in Eigen).
using ColumnMajorXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

}  // namespace acados

#endif  // ACADOS_SOLVER_BASE__ACADOS_TYPES_HPP_
