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
#include <mock_acados_solver/mock_acados_solver.hpp>

TEST(TestCreateMockSolver, test_init)
{
  int N = 10;
  double Ts = 0.01;

  acados::MockAcadosSolver solver;
  ASSERT_NO_THROW(solver.init(N, Ts));
  ASSERT_EQ(solver.N(), N);
  ASSERT_EQ(solver.Ts(), Ts);
}
TEST(TestCreateMockSolver, test_get_NLP_dimensions)
{
  acados::MockAcadosSolver solver;
  ASSERT_NO_THROW(solver.init(10, 0.1));
  ASSERT_EQ(solver.dims().nx, 4);
  ASSERT_EQ(solver.dims().nz, 0);
  ASSERT_EQ(solver.dims().nu, 1);
  ASSERT_EQ(solver.dims().np, 2);
}
