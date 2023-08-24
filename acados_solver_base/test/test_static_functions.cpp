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

#include <gtest/gtest.h>
#include "acados_solver_base/acados_solver.hpp"


// Test mapping utils
TEST(TestStaticFunctions, test_fill_vector_from_map)
{
  acados::ValueMap value_map;
  value_map["a"] = std::vector<double>{1.0, 2.0};
  value_map["b"] = std::vector<double>{3.0};
  value_map["c"] = std::vector<double>{4.0};

  acados::IndexMap index_map {{"a", {0, 1}}, {"b", {3}}, {"c", {2}}};
  std::vector<double> values;
  acados::AcadosSolver::fill_vector_from_map(index_map, value_map, 4, values);
  // Should return {1.0, 2.0, 4.0, 3.0}
  ASSERT_EQ(values[0], 1.0);
  ASSERT_EQ(values[1], 2.0);
  ASSERT_EQ(values[2], 4.0);
  ASSERT_EQ(values[3], 3.0);
}
TEST(TestStaticFunctions, test_create_map_from_values)
{
  acados::IndexMap index_map {{"a", {0, 1, 2}}, {"b", {5}}, {"c", {3, 4}}};
  std::vector<double> values {1.0, 2.0, 4.0, 3.0, 5.0, 6.0};
  acados::ValueMap returned_value_map = acados::AcadosSolver::create_map_from_values(
    index_map,
    values);

  acados::ValueMap expected_value_map {{"a", {1.0, 2.0, 4.0}}, {"b", {6.0}}, {"c", {3.0, 5.0}}};

  ASSERT_EQ(returned_value_map.size(), expected_value_map.size());
  ASSERT_EQ((returned_value_map == expected_value_map), true);
}

TEST(TestStaticFunctions, test_is_map_size_consistent)
{
  acados::ValueMap value_map {{"a", {0, 1, 2}}, {"b", {3}}, {"c", {4, 5}}};  // size 6
  acados::IndexMap index_map {{"d", {0, 1}}, {"e", {2}}, {"f", {3}}};  // size 4
  ASSERT_EQ(acados::AcadosSolver::is_map_size_consistent(value_map, 6), true);
  ASSERT_EQ(acados::AcadosSolver::is_map_size_consistent(value_map, 7), false);
  ASSERT_EQ(acados::AcadosSolver::is_map_size_consistent(value_map, 5), false);
  ASSERT_EQ(acados::AcadosSolver::is_map_size_consistent(index_map, 4), true);
  ASSERT_EQ(acados::AcadosSolver::is_map_size_consistent(index_map, 5), false);
  ASSERT_EQ(acados::AcadosSolver::is_map_size_consistent(index_map, 3), false);
}
TEST(TestStaticFunctions, test_is_values_map_complete)
{
  acados::IndexMap index_map {{"a", {0, 1, 2}}, {"b", {5}}, {"c", {3, 4}}};

  acados::ValueMap complete_value_map {{"a", {0, 1, 2}}, {"b", {3}}, {"c", {4, 5}}};
  acados::ValueMap redundant_value_map {{"a", {0, 1, 2}}, {"b", {3}}, {"c", {4, 5}}};
  acados::ValueMap incomplete_value_map_1 {{"a", {0, 1, 2}}, {"b", {5}}};
  acados::ValueMap incomplete_value_map_2 {{"a", {0, 1, 2}}, {"b", {5}}, {"c", {4}}};

  ASSERT_EQ(acados::AcadosSolver::is_values_map_complete(index_map, complete_value_map), true);
  ASSERT_EQ(acados::AcadosSolver::is_values_map_complete(index_map, redundant_value_map), true);
  ASSERT_EQ(acados::AcadosSolver::is_values_map_complete(index_map, incomplete_value_map_1), false);
  ASSERT_EQ(acados::AcadosSolver::is_values_map_complete(index_map, incomplete_value_map_2), false);
}
