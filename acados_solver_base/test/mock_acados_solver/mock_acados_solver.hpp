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

#pragma once

#include "acados_solver_base/acados_solver.hpp"

struct mock_acados_solver_solver_capsule;

namespace acados
{
class MockAcadosSolver : public acados::AcadosSolver
{
public:
  // Constructor
  MockAcadosSolver();
  ~MockAcadosSolver();

protected:
  virtual int create_index_maps();
  virtual int internal_create_capsule();
  virtual int internal_create_with_discretization(
    int n_time_steps,
    double * new_time_steps);
  virtual int internal_reset(int reset_qp_solver_mem);
  virtual int internal_free();
  virtual int internal_free_capsule();

  virtual int internal_update_qp_solver_cond_N(int qp_solver_cond_N);
  virtual int internal_update_params(unsigned int stage, double * value, int np);
  virtual int internal_update_params_sparse(unsigned int stage, int * idx, double * p, int n_update);
  virtual int internal_solve();
  virtual void internal_print_stats() const;

  virtual ocp_nlp_in * get_nlp_in() const;
  virtual ocp_nlp_out * get_nlp_out() const;
  virtual ocp_nlp_out * get_sens_out() const;
  virtual ocp_nlp_solver * get_nlp_solver() const;
  virtual ocp_nlp_config * get_nlp_config() const;
  virtual void * get_nlp_opts() const;
  virtual ocp_nlp_dims * get_nlp_dims() const;
  virtual ocp_nlp_plan_t * get_nlp_plan() const;
  virtual unsigned int get_nlp_np() const;

private:
  mock_acados_solver_solver_capsule * _capsule = nullptr;
};

} // namespace acados