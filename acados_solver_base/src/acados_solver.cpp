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

#include "acados_solver_base/acados_solver.hpp"
#include <numeric>  // for std::iota
#include <stdexcept>

namespace acados
{

AcadosSolver::AcadosSolver()
{
}

AcadosSolver::~AcadosSolver()
{
}

int AcadosSolver::init(int N, double Ts)
{
  // Create capsule
  int status = internal_create_capsule();
  // Init solver
  std::vector<double> time_steps_vect(N, Ts);
  status = internal_create_with_discretization(N, time_steps_vect.data());
  _Ts = Ts;

  // Create index maps
  status = create_index_maps();
  if (!is_map_size_consistent(_x_index_map, nx())) {
    status = -1;
    std::cout << "Inconsistent index map for diff. state variables x!" << std::endl;
  }
  if (!is_map_size_consistent(_z_index_map, nz())) {
    status = -1;
    std::cout << "Inconsistent index map for algebraic state variables z!" << std::endl;
  }
  if (!is_map_size_consistent(_p_index_map, np())) {
    status = -1;
    std::cout << "Inconsistent index map for runtime parameters variables p!" << std::endl;
  }
  if (!is_map_size_consistent(_u_index_map, nu())) {
    status = -1;
    std::cout << "Inconsistent index map for control variables u!" << std::endl;
  }
  if (status < 1) {
    std::cout << "ERROR: the index maps could not be initialized correctly!" << std::endl;
    return -1;
  }

  return reset();
}

int AcadosSolver::reset()
{
  _rti_phase = 0;
  return internal_reset(1);
}


int AcadosSolver::free_memory()
{
  int status = internal_free();
  // TODO(tpoignonec): test all OK
  status = internal_free_capsule();
  // TODO(tpoignonec): test all OK
  return status;
}

int AcadosSolver::solve()
{
  ocp_nlp_solver_opts_set(get_nlp_config(), get_nlp_opts(), "rti_phase", &_rti_phase);
  int solver_status = internal_solve();
  if (solver_status != ACADOS_SUCCESS) {
    std::cout << "WARNING! AcadosSolver::solve() failed with status " << solver_status << '!' <<
      std::endl;
  }
  return solver_status;
}


//####################################################
//                     SETTERS
//####################################################

// ------------------------------------------
// Initial state values
// ------------------------------------------
int AcadosSolver::set_initial_state_values(std::vector<double> & x_0)
{
  if (static_cast<int>(x_0.size()) != nx()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_initial_state_values()': "
      "Inconsistent parameters, the size of x_0 should match nx!";
    throw std::range_error(err_msg);
  }
  std::vector<int> idxbx0(nx());
  std::iota(std::begin(idxbx0), std::end(idxbx0), 0);    // Fill with 0, 1, ..., nx()-1.
  set_state_bounds(0, idxbx0, x_0, x_0);
  return 0;
}

int AcadosSolver::set_initial_state_values(ValueMap const & x_0_map)
{
  if (!is_values_map_complete(x_index_map(), x_0_map)) {
    return -1;
  }
  std::vector<double> x_0;
  x_0.reserve(nx());
  fill_vector_from_map(x_index_map(), x_0_map, nx(), x_0);
  return set_initial_state_values(x_0);
}

// ------------------------------------------
// Bounds and constraints
// ------------------------------------------
int AcadosSolver::set_state_bounds(
  int stage, std::vector<int> & idxbx, std::vector<double> & lbx,
  std::vector<double> & ubx)
{
  if (stage < 0 || stage > N()) {
    std::string err_msg = "Error in 'AcadosSolver::set_state_bounds()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  if (idxbx.size() != lbx.size() || idxbx.size() != ubx.size() || lbx.size() != ubx.size()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_state_bounds()': "
      "Inconsistent parameters (idxbx, lbx, and ubx should have the same length)!";
    throw std::range_error(err_msg);
  }
  int expected_dim;
  if (stage == 0) {
    expected_dim = dims().nbx0;
  } else if (stage < N()) {
    expected_dim = dims().nbx;
  } else {
    expected_dim = dims().nbxN;
  }
  if (static_cast<int>(idxbx.size()) > expected_dim) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_state_bounds()': "
      "Inconsistent parameters! The size of idxbx, lbx, or ubx cannot be of length ";
    err_msg += std::to_string(expected_dim);
    throw std::range_error(err_msg);
  }
  ocp_nlp_constraints_model_set(
    get_nlp_config(), get_nlp_dims(),
    get_nlp_in(), stage, "idxbx", idxbx.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(), get_nlp_dims(),
    get_nlp_in(), stage, "lbx", lbx.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(), get_nlp_dims(),
    get_nlp_in(), stage, "ubx", ubx.data());
  return 0;
}
int AcadosSolver::set_state_bounds(
  std::vector<int> & idxbx, std::vector<double> & lbx,
  std::vector<double> & ubx)
{
  // From 0 to N as there is no control at terminal node!
  for (int stage = 0; stage < N(); stage++) {
    set_state_bounds(stage, idxbx, lbx, ubx);
  }
  return 0;
}

int AcadosSolver::set_control_bounds(
  int stage, std::vector<int> & idxbu, std::vector<double> & lbu,
  std::vector<double> & ubu)
{
  if (stage < 0 || stage > N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_control_bounds()': "
      "Invalid stage request!";
    throw std::range_error(err_msg);
  }
  if (idxbu.size() != lbu.size() || idxbu.size() != ubu.size() || lbu.size() != ubu.size()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_control_bounds()': "
      "Inconsistent parameters (idxbu, lbu, and ubu should have the same length)!";
    throw std::range_error(err_msg);
  }
  if (static_cast<int>(idxbu.size()) != dims().nbu) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_control_bounds()': "
      "Inconsistent parameters (the size of idxbu, lbu, or ubu should be of length ";
    err_msg += std::to_string(dims().nbu);
    throw std::range_error(err_msg);
  }
  ocp_nlp_constraints_model_set(
    get_nlp_config(), get_nlp_dims(),
    get_nlp_in(), stage, "idxbu", idxbu.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(), get_nlp_dims(),
    get_nlp_in(), stage, "lbu", lbu.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(), get_nlp_dims(),
    get_nlp_in(), stage, "ubu", ubu.data());
  return 0;
}
int AcadosSolver::set_control_bounds(
  std::vector<int> & idxbu, std::vector<double> & lbu,
  std::vector<double> & ubu)
{
  // From 0 to N-1 as there is no control at terminal node!
  for (int stage = 0; stage < N(); stage++) {
    set_control_bounds(stage, idxbu, lbu, ubu);
  }
  return 0;
}

// ------------------------------------------
// Solver state/control values initialization
// ------------------------------------------
int AcadosSolver::initialize_state_values(int stage, std::vector<double> & x_i)
{
  if (static_cast<int>(x_i.size()) != nx()) {
    std::cout << "WARNING: Failed to set x_" << stage << "!"
              << "A vector of length " << nx() << " is expected (" << x_i.size() << " provided)." <<
      std::endl;
    return -1;
  }
  if (stage < 0 || stage > N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::initialize_state_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  ocp_nlp_out_set(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "x", x_i.data());
  return 0;
}
int AcadosSolver::initialize_state_values(int stage, ValueMap const & x_i_map)
{
  if (!is_values_map_complete(x_index_map(), x_i_map)) {
    return -1;
  }
  std::vector<double> x_i;
  x_i.reserve(nx());
  fill_vector_from_map(x_index_map(), x_i_map, nx(), x_i);
  return initialize_state_values(stage, x_i);
}

int AcadosSolver::initialize_state_values(std::vector<double> & x_i)
{
  int status = 0;
  // From 0 to N to also set terminal state!
  for (int stage = 0; stage <= N(); stage++) {
    status += initialize_state_values(stage, x_i);
  }
  return 0;
}

int AcadosSolver::initialize_state_values(ValueMap const & x_i_map)
{
  if (!is_values_map_complete(x_index_map(), x_i_map)) {
    return -1;
  }
  std::vector<double> x_i;
  x_i.reserve(nx());
  fill_vector_from_map(x_index_map(), x_i_map, nx(), x_i);
  return initialize_state_values(x_i);
}

int AcadosSolver::initialize_control_values(int stage, std::vector<double> & u_i)
{
  if (static_cast<int>(u_i.size()) != nu()) {
    std::cout << "WARNING: Failed to set u_" << stage << "!"
              << "A vector of length " << nu() << " is expected (" << u_i.size() << " provided)." <<
      std::endl;
    return -1;
  }
  if (stage < 0 || stage >= N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::initialize_control_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  ocp_nlp_out_set(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "u", u_i.data());
  return 0;
}
int AcadosSolver::initialize_control_values(int stage, ValueMap const & u_i_map)
{
  if (!is_values_map_complete(u_index_map(), u_i_map)) {
    return -1;
  }
  std::vector<double> u_i;
  u_i.reserve(nu());
  fill_vector_from_map(u_index_map(), u_i_map, nx(), u_i);
  return initialize_control_values(stage, u_i);
}
int AcadosSolver::initialize_control_values(std::vector<double> & u_i)
{
  int status = 0;
  // From 0 to N-1 as there is no control at terminal node!
  for (int stage = 0; stage < N(); stage++) {
    status += initialize_control_values(stage, u_i);
  }
  return status;
}
int AcadosSolver::initialize_control_values(ValueMap const & u_i_map)
{
  if (!is_values_map_complete(u_index_map(), u_i_map)) {
    return -1;
  }
  std::vector<double> u_i;
  u_i.reserve(nu());
  fill_vector_from_map(u_index_map(), u_i_map, nx(), u_i);
  return initialize_control_values(u_i);
}

// ------------------------------------------
// Runtime parameters
// ------------------------------------------
int AcadosSolver::set_runtime_parameters(int stage, std::vector<double> & p_i)
{
  if (static_cast<int>(p_i.size()) != np()) {
    std::cout << "WARNING: Failed to set p_" << stage << "!"
              << "A vector of length " << np() << " is expected (" << p_i.size() << " provided)." <<
      std::endl;
    return -1;
  }
  if (stage < 0 || stage > N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_runtime_parameters()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  return internal_update_params(stage, p_i.data(), np());
}
int AcadosSolver::set_runtime_parameters(int stage, ValueMap const & p_i_map)
{
  if (!is_values_map_complete(p_index_map(), p_i_map)) {
    return -1;
  }
  std::vector<double> p_i;
  p_i.reserve(np());
  fill_vector_from_map(p_index_map(), p_i_map, np(), p_i);
  return initialize_control_values(stage, p_i);
}
int AcadosSolver::set_runtime_parameters(std::vector<double> & p_i)
{
  // From 0 to N to also set terminal node parameters!
  int status = 0;
  for (int stage = 0; stage <= N(); stage++) {
    status += set_runtime_parameters(stage, p_i);
  }
  return 0;
}
int AcadosSolver::set_runtime_parameters(ValueMap const & p_i_map)
{
  if (!is_values_map_complete(p_index_map(), p_i_map)) {
    return -1;
  }
  std::vector<double> p_i;
  p_i.reserve(np());
  fill_vector_from_map(p_index_map(), p_i_map, np(), p_i);
  return initialize_control_values(p_i);
}


//####################################################
//                     GETTERS
//####################################################

std::vector<double> AcadosSolver::get_state_values(int stage)
{
  if (stage < 0 || stage > N()) {
    std::string err_msg = "Error in 'AcadosSolver::get_state_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  std::vector<double> x_i(nx(), 0.0);
  ocp_nlp_out_get(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "x", x_i.data());
  return x_i;
}
std::vector<double> AcadosSolver::get_algebraic_state_values(int stage)
{
  if (stage < 0 || stage >= N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::get_algebraic_state_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  std::vector<double> z_i(nz(), 0.0);
  ocp_nlp_out_get(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "z", z_i.data());
  return z_i;
}
std::vector<double> AcadosSolver::get_control_values(int stage)
{
  if (stage < 0 || stage >= N()) {
    std::string err_msg = "Error in 'AcadosSolver::get_control_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  std::vector<double> u_i(nu(), 0.0);
  ocp_nlp_out_get(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "u", u_i.data());
  return u_i;
}

const IndexMap & AcadosSolver::x_index_map() const
{
  return _x_index_map;
}
const IndexMap & AcadosSolver::z_index_map() const
{
  return _z_index_map;
}
const IndexMap & AcadosSolver::p_index_map() const
{
  return _p_index_map;
}
const IndexMap & AcadosSolver::u_index_map() const
{
  return _u_index_map;
}


//####################################################
//                Map management utils
//####################################################

bool AcadosSolver::is_map_size_consistent(IndexMap const & map, int expected_total_values)
{
  int total_values = 0;
  for (const auto & [key, value] : map) {
    total_values += value.size();
  }
  return total_values == expected_total_values;
}
bool AcadosSolver::is_map_size_consistent(ValueMap const & map, int expected_total_values)
{
  int total_values = 0;
  for (const auto & [key, value] : map) {
    total_values += value.size();
  }
  return total_values == expected_total_values;
}

bool AcadosSolver::is_values_map_complete(
  IndexMap const & index_map,
  ValueMap const & values_map)
{
  bool all_ok = true;
  for (const auto & [key, indexes] : index_map) {
    if (values_map.find(key) == values_map.end()) {
      std::cout << "key '" << key << "' not found!" << std::endl;
      all_ok = false;
      break;
    }
    all_ok &= (indexes.size() == values_map.at(key).size());
    if (!all_ok) {
      std::cout << "key '" << key << "' has values of incorrect size!" << std::endl;
    }
    if (!all_ok) {
      break;
    }
  }
  return all_ok;
}
void AcadosSolver::fill_vector_from_map(
  IndexMap const & index_map,
  ValueMap const & values_map,
  unsigned int vector_size,
  std::vector<double> & values)
{
  // Resize container if needed
  if (values.size() != vector_size) {
    values.resize(vector_size);
  }
  // Check values map is complete
  if (!is_values_map_complete(index_map, values_map)) {
    // TODO(tpoignonec): throw missing key name ?
    throw std::invalid_argument("Incomplete map provided to 'fill_vector_from_map()'!");
  }
  // Fill the vector
  for (const auto & [key, indexes] : index_map) {
    std::vector<double> input_values = values_map.at(key);
    for (unsigned int i = 0; i < indexes.size(); i++) {
      values[indexes[i]] = input_values[i];
    }
  }
}
ValueMap AcadosSolver::create_map_from_values(
  IndexMap const & index_map,
  std::vector<double> const & values)
{
  if (!is_map_size_consistent(index_map, static_cast<int>(values.size()))) {
    throw std::invalid_argument("Inconsistent data provided to 'create_map_from_values()'!");
  }
  ValueMap value_map;
  // Fill the map
  for (const auto & [key, indexes] : index_map) {
    std::vector<double> tmp_data(indexes.size(), 0.0);
    for (unsigned int i = 0; i < indexes.size(); i++) {
      tmp_data[i] = values[indexes[i]];
    }
    value_map[key] = tmp_data;
  }
  return value_map;
}

//####################################################
//                  DIMENSIONS
//####################################################

const AcadosSolver::Dimensions & AcadosSolver::dims() const
{
  return _dims;
}
int AcadosSolver::nx() const
{
  // return *(get_nlp_dims()->nx);
  return dims().nx;
}
int AcadosSolver::nz() const
{
  // return *(get_nlp_dims()->nz);
  return dims().nz;
}
int AcadosSolver::np() const
{
  // return static_cast<int>(get_nlp_np());
  return dims().np;
}
int AcadosSolver::nu() const
{
  // return *(get_nlp_dims()->nu);
  return dims().nu;
}
int AcadosSolver::N() const
{
  return get_nlp_dims()->N;
}
double AcadosSolver::Ts() const
{
  return _Ts;
}
std::vector<double> AcadosSolver::sampling_intervals() const
{
  std::vector<double> sampling_intervals_vect;
  sampling_intervals_vect.assign(get_nlp_in()->Ts, get_nlp_in()->Ts + N());
  return sampling_intervals_vect;
}

}  // namespace acados
