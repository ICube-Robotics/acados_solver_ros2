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

#include <iostream>
#include <string>
#include <unordered_map>

#include <cstdio>

#include <pluginlib/class_loader.hpp>
#include "acados_solver_base/acados_solver.hpp"


template<class T>
void print_vector(std::vector<T> const & input)
{
  std::cout << "[ ";
  for (auto const & i : input) {
    std::cout << i << " ";
  }
  std::cout << "]";
}

template<class T>
void print_map(std::unordered_map<std::string, std::vector<T>> const & map)
{
  std::cout << "{ ";
  for (const auto & [key, value] : map) {
    std::cout << key << " = ";
    print_vector(value);
    std::cout << ", ";
  }
  std::cout << " }\n";
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::string solver_plugin_name = "acados_solver_plugins_example/MockAcadosSolver";

  pluginlib::ClassLoader<acados::AcadosSolver> acados_solver_loader("acados_solver_base",
    "acados::AcadosSolver");
  std::shared_ptr<acados::AcadosSolver> solver = acados_solver_loader.createSharedInstance(
    solver_plugin_name);


  std::cout << "Loading solver plugin \"" << solver_plugin_name << "\"" << std::endl;

  int N = 10;
  double Ts = 0.01;
  std::cout << "Initializing solver with N = " << N << " and Ts = " << Ts << ":" << std::endl;
  solver->init(N, Ts);

  std::cout << "Getting problem dimensions:" << std::endl;
  std::cout << "   nx = " << solver->nx() << std::endl;
  std::cout << "   nz = " << solver->nz() << std::endl;
  std::cout << "   np = " << solver->np() << std::endl;
  std::cout << "   nu = " << solver->nu() << std::endl;
  std::cout << "   N  = " << solver->N() << std::endl;
  std::cout << "   Ts = " << solver->Ts() << std::endl;
  std::cout << "   Sampling intervals = ";
  print_vector(solver->sampling_intervals());
  std::cout << std::endl << std::endl;

  std::cout << "Retrieve index mappings : " << std::endl;
  std::cout << "   index map x = "; print_map(solver->x_index_map());
  std::cout << "   index map z = "; print_map(solver->z_index_map());
  std::cout << "   index map p = "; print_map(solver->p_index_map());
  std::cout << "   index map u = "; print_map(solver->u_index_map());

  // Test map utils
  acados::ValueMap x_values_map;
  x_values_map["p"] = acados::ValueVector{1.0};
  x_values_map["p_dot"] = acados::ValueVector{0.0};
  x_values_map["theta"] = acados::ValueVector{3.1415};
  x_values_map["theta_dot"] = acados::ValueVector{0.0};
  x_values_map["useless_key_for_test"] = acados::ValueVector{0.0, 10.0};

  std::cout << "Provided values_map = " << std::endl;
  print_map(x_values_map);
  acados::ValueVector x_values;
  acados::AcadosSolver::fill_vector_from_map(solver->x_index_map(), x_values_map, 4, x_values);
  std::cout << "Generated values = ";
  print_vector(x_values);
  std::cout << std::endl << std::endl;

  std::cout << "Test set p :" << std::endl;
  acados::ValueVector p_i {
    1.0,  // Cart mass
    0.1   // Ball mass
  };
  for (int idx = 0; idx <= N; idx++) {
    solver->set_runtime_parameters(idx, p_i);
  }

  std::cout << "Set initial state = [0, pi, 0, 0]" << std::endl;
  {
    acados::ValueVector x0 {0., 3.14, 0.0, 0.0};
    solver->set_initial_state_values(x0);
    solver->initialize_state_values(x0);
  }
  for (int idx = 0; idx <= N; idx++) {
    std::cout << "  x_" << idx << " = ";
    print_vector(solver->get_state_values(idx));
    std::cout << std::endl;
  }

  std::cout << "Set some u bounds (nbu = " << solver->dims().nbu << "):" << std::endl;
  acados::IndexVector idxbu = {0};
  acados::ValueVector lbu = {-10};
  acados::ValueVector ubu = {10};
  for (int idx = 0; idx < N; idx++) {
    solver->set_control_bounds(idx, idxbu, lbu, ubu);
  }

  std::cout << "Set some x bounds (nbx = " << solver->dims().nbx << "):" << std::endl;
  acados::IndexVector idxbx = {2};
  acados::ValueVector lbx = {0.0};
  acados::ValueVector ubx = {0.5};
  for (int idx = 1; idx <= N; idx++) {
    solver->set_state_bounds(idx, idxbx, lbx, ubx);
  }


  std::cout << "Calling 'solve()' ..." << std::endl;
  int solver_status = solver->solve();
  std::cout << "         ... returned status = " << solver_status << std::endl;


  std::cout << "Prediction over horizon :" << std::endl;
  for (int idx = 0; idx <= N; idx++) {
    std::cout << "  x_" << idx << " = ";
    print_vector(solver->get_state_values(idx));
    std::cout << std::endl;
  }
  for (int idx = 0; idx < N; idx++) {
    std::cout << "  z_" << idx << " = ";
    print_vector(solver->get_algebraic_state_values(idx));
    std::cout << std::endl;
  }
  for (int idx = 0; idx < N; idx++) {
    std::cout << "  u_" << idx << " = ";
    print_vector(solver->get_control_values(idx));
    std::cout << std::endl;
  }

  std::cout << "Test create_map_from_values :" << std::endl;
  std::cout << "x_pred(i=6) = ";
  acados::ValueMap x_values_map_idx6 = acados::AcadosSolver::create_map_from_values(
    solver->x_index_map(), solver->get_state_values(6));
  print_map(x_values_map_idx6);
  return 0;
}
