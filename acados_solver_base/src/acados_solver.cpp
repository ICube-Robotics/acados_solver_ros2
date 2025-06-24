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

int AcadosSolver::init(unsigned int N, double Ts)
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
    status = 1;
    std::cerr << "Inconsistent index map for diff. state variables x!" << std::endl;
  }
  if (!is_map_size_consistent(_z_index_map, nz())) {
    status = 1;
    std::cerr << "Inconsistent index map for algebraic state variables z!" << std::endl;
  }
  if (!is_map_size_consistent(_p_index_map, np())) {
    status = 1;
    std::cerr << "Inconsistent index map for runtime parameters variables p!" << std::endl;
  }
  if (!is_map_size_consistent(_u_index_map, nu())) {
    status = 1;
    std::cerr << "Inconsistent index map for control variables u!" << std::endl;
  }
  if (status > 0) {
    std::cerr << "ERROR: the index maps could not be initialized correctly!" << std::endl;
    return 1;
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
  bool use_rti = get_nlp_config()->is_real_time_algorithm();
  int solver_status = -1;

  if (use_rti) {
    // RTI preparation stage
    solver_status = solve_rti(RtiStage::PREPARATION);
    if (solver_status != ACADOS_READY && solver_status != ACADOS_SUCCESS) {
      std::cerr << "Aborting RTI solve() at preparation stage!" << std::endl;
      return solver_status;
    }
    // RTI feedback stage
    solver_status = solve_rti(RtiStage::FEEDBACK);
  } else {
    // Vanilla solve
    solver_status = internal_solve();
  }

  if (solver_status != ACADOS_SUCCESS) {
    std::cerr << "WARNING! AcadosSolver::solve() failed with status " << solver_status << '!' <<
      std::endl;
    std::cerr << "Realtime mode: " << (use_rti ? "ON" : "OFF") << std::endl;
  }
  return solver_status;
}

int AcadosSolver::solve_rti(RtiStage rti_phase)
{
  int rti_status = -1;

  // RTI initialization stage (optional)
  if (_rti_phase < 0) {
    std::cout << "Attempting to initialize the solver with RTI!" << std::endl;
    for (size_t attempt_nb = 0; attempt_nb < 10; attempt_nb++) {
      rti_status = internal_solve();
    }
    if (rti_status != ACADOS_READY && rti_status != ACADOS_SUCCESS) {
      std::cerr << "WARNING! AcadosSolver::solve() failed to initialize with status " <<
        rti_status << '!' <<
        std::endl;
      return rti_status;
    }
  }

  if (rti_phase != RtiStage::PREPARATION && rti_phase != RtiStage::FEEDBACK) {
    std::cerr << "ERROR! AcadosSolver::solve_rti() called with an invalid RTI phase!" << std::endl;
    return -1;  // Not standard Acados status code...
  }

  if (rti_phase == RtiStage::PREPARATION) {
    _rti_phase = 1;
    ocp_nlp_solver_opts_set(get_nlp_config(), get_nlp_opts(), "rti_phase", &_rti_phase);
    rti_status = internal_solve();
    if (rti_status != ACADOS_READY && rti_status != ACADOS_SUCCESS) {
      std::cerr <<
        "WARNING! AcadosSolver::solve() failed during RTI preparation stage with status " <<
        rti_status << '!' <<
        std::endl;
      return rti_status;
    }
  } else {
    // RTI feedback stage
    _rti_phase = 2;
    ocp_nlp_solver_opts_set(get_nlp_config(), get_nlp_opts(), "rti_phase", &_rti_phase);
    rti_status = internal_solve();
    if (rti_status != ACADOS_SUCCESS) {
      std::cerr << "WARNING! AcadosSolver::solve() failed during RTI feedback stage with status " <<
        rti_status << '!' <<
        std::endl;
      return rti_status;
    }
  }
  return rti_status;
}

//####################################################
//                  SIMULATION
//####################################################

int AcadosSolver::simulate(
  double dt,
  ValueVector & x0,
  ValueVector & u0,
  ValueVector & p,
  ValueVector & x_next,
  ValueVector & z)
{
  if (dt <= 0.0) {
    std::cerr << "Error in 'AcadosSolver::simulate()': Invalid time step, got " << dt << std::endl;
    return 10;  // Error: Invalid time step
  }
  if (x0.size() != nx() || u0.size() != nu() || p.size() != np()) {
    std::cerr << "Error in 'AcadosSolver::simulate()': Inconsistent parameters!" << std::endl;
    return 11;  // Error: Inconsistent parameters
  }
  if (x_next.size() != nx() || z.size() != nz()) {
    std::cerr << "Error in 'AcadosSolver::simulate()': Inconsistent output sizes!" << std::endl;
    return 12;  // Error: Inconsistent output sizes
  }

  return internal_simulate(dt, x0.data(), u0.data(), p.data(), x_next.data(), z.data());
}

  /**
   * @brief Simulate the next state given the current state, control inputs and runtime parameters.
   *
   * See the other `simulate()` method for details.
   */
int AcadosSolver::simulate(
  double dt,
  ValueMap const & x0_map,
  ValueMap const & u0_map,
  ValueMap const & p_map,
  ValueMap & x_next_map,
  ValueMap & z_map)
{
  if (!is_values_map_complete(x_index_map(), x0_map) ||
    !is_values_map_complete(u_index_map(), u0_map) ||
    !is_values_map_complete(p_index_map(), p_map))
  {
    return 1;  // Error: Incomplete values map
  }
  std::vector<double> x0, u0, p;
  x0.reserve(nx());
  u0.reserve(nu());
  p.reserve(np());
  fill_vector_from_map(x_index_map(), x0_map, nx(), x0);
  fill_vector_from_map(u_index_map(), u0_map, nu(), u0);
  fill_vector_from_map(p_index_map(), p_map, np(), p);
  std::vector<double> x_next(nx()), z(nz());

  int status = simulate(dt, x0, u0, p, x_next, z);
  fill_map_from_values(x_index_map(), x_next, x_next_map);
  fill_map_from_values(z_index_map(), z, z_map);

  if (status != 0) {
    std::cerr << "Error in 'AcadosSolver::simulate()': Simulation failed with status " << status <<
      std::endl;
  }

  return status;  // Error: Simulation failed
}


int AcadosSolver::simulate(
  ValueMap const & x0_map,
  ValueMap const & u0_map,
  ValueMap const & p_map,
  ValueMap & x_next_map,
  ValueMap & z_map)
{
  return simulate(
    Ts(),
    x0_map,
    u0_map,
    p_map,
    x_next_map,
    z_map);
}

//####################################################
//                     SETTERS
//####################################################

// ------------------------------------------
// Initial state values
// ------------------------------------------
int AcadosSolver::set_initial_state_values(ValueVector & x_0)
{
  if (x_0.size() != nx()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_initial_state_values()': "
      "Inconsistent parameters, the size of x_0 should match nx!";
    throw std::range_error(err_msg);
  }
  IndexVector idxbx0(nx());
  std::iota(std::begin(idxbx0), std::end(idxbx0), 0);    // Fill with 0, 1, ..., nx()-1.
  set_state_bounds(0, idxbx0, x_0, x_0);
  return 0;
}

int AcadosSolver::set_initial_state_values(ValueMap const & x_0_map)
{
  if (!is_values_map_complete(x_index_map(), x_0_map)) {
    return 1;
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
  unsigned int stage,
  IndexVector & idxbx,
  ValueVector & lbx,
  ValueVector & ubx)
{
  if (stage > N()) {
    std::string err_msg = "Error in 'AcadosSolver::set_state_bounds()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  if (idxbx.size() != lbx.size() || idxbx.size() != ubx.size() || lbx.size() != ubx.size()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_state_bounds()': "
      "Inconsistent parameters (idxbx, lbx, and ubx should have the same length)!";
    throw std::range_error(err_msg);
  }
  unsigned int expected_dim;
  if (stage == 0) {
    expected_dim = dims().nbx_0;
  } else if (stage < N()) {
    expected_dim = dims().nbx;
  } else {
    expected_dim = dims().nbx_N;
  }
  if (idxbx.size() != expected_dim) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_state_bounds()': "
      "Inconsistent parameters! The size of idxbx, lbx, and ubx shoulb be of dimension ";
    err_msg += std::to_string(expected_dim);
    throw std::range_error(err_msg);
  }
  ocp_nlp_constraints_model_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_in(),
    get_nlp_out(),
    stage, "idxbx", idxbx.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_in(),
    get_nlp_out(),
    stage, "lbx", lbx.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_in(),
    get_nlp_out(),
    stage, "ubx", ubx.data());
  return 0;
}

int AcadosSolver::set_state_bounds(IndexVector & idxbx, ValueVector & lbx, ValueVector & ubx)
{
  // From 0 to N as there is no control at terminal node!
  for (unsigned int stage = 0; stage < N(); stage++) {
    set_state_bounds(stage, idxbx, lbx, ubx);
  }
  return 0;
}

int AcadosSolver::set_control_bounds(
  unsigned int stage,
  IndexVector & idxbu,
  ValueVector & lbu,
  ValueVector & ubu)
{
  if (stage > N()) {
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
  if (idxbu.size() != dims().nbu) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_control_bounds()': "
      "Inconsistent parameters (the size of idxbu, lbu, or ubu should be of length ";
    err_msg += std::to_string(dims().nbu);
    throw std::range_error(err_msg);
  }
  ocp_nlp_constraints_model_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_in(),
    get_nlp_out(),
    stage, "idxbu", idxbu.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_in(),
    get_nlp_out(),
    stage, "lbu", lbu.data());
  ocp_nlp_constraints_model_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_in(),
    get_nlp_out(),
    stage, "ubu", ubu.data());
  return 0;
}

int AcadosSolver::set_control_bounds(IndexVector & idxbu, ValueVector & lbu, ValueVector & ubu)
{
  // From 0 to N-1 as there is no control at terminal node!
  for (unsigned int stage = 0; stage < N(); stage++) {
    set_control_bounds(stage, idxbu, lbu, ubu);
  }
  return 0;
}

// ------------------------------------------
// Solver state/control values initialization
// ------------------------------------------
int AcadosSolver::initialize_state_values(unsigned int stage, ValueVector & x_i)
{
  if (x_i.size() != nx()) {
    std::cerr << "WARNING: Failed to set x_" << stage << "!"
              << "A vector of length " << nx() << " is expected (" << x_i.size() << " provided)." <<
      std::endl;
    return 1;
  }
  if (stage > N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::initialize_state_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  ocp_nlp_out_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_out(),
    get_nlp_in(),
    stage, "x", x_i.data()
  );
  return 0;
}

int AcadosSolver::initialize_state_values(unsigned int stage, ValueMap const & x_i_map)
{
  if (!is_values_map_complete(x_index_map(), x_i_map)) {
    return 1;
  }
  std::vector<double> x_i;
  x_i.reserve(nx());
  fill_vector_from_map(x_index_map(), x_i_map, nx(), x_i);
  return initialize_state_values(stage, x_i);
}

int AcadosSolver::initialize_state_values(ValueVector & x_i)
{
  int status = 0;
  // From 0 to N to also set terminal state!
  for (unsigned int stage = 0; stage <= N(); stage++) {
    status += initialize_state_values(stage, x_i);
  }
  return 0;
}

int AcadosSolver::initialize_state_values(ValueMap const & x_i_map)
{
  if (!is_values_map_complete(x_index_map(), x_i_map)) {
    return 1;
  }
  std::vector<double> x_i;
  x_i.reserve(nx());
  fill_vector_from_map(x_index_map(), x_i_map, nx(), x_i);
  return initialize_state_values(x_i);
}

int AcadosSolver::initialize_control_values(unsigned int stage, ValueVector & u_i)
{
  if (u_i.size() != nu()) {
    std::cerr << "WARNING: Failed to set u_" << stage << "!"
              << "A vector of length " << nu() << " is expected (" << u_i.size() << " provided)." <<
      std::endl;
    return 1;
  }
  if (stage >= N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::initialize_control_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  ocp_nlp_out_set(
    get_nlp_config(),
    get_nlp_dims(),
    get_nlp_out(),
    get_nlp_in(),
    stage, "u", u_i.data()
  );
  return 0;
}
int AcadosSolver::initialize_control_values(unsigned int stage, ValueMap const & u_i_map)
{
  if (!is_values_map_complete(u_index_map(), u_i_map)) {
    return 1;
  }
  std::vector<double> u_i;
  u_i.reserve(nu());
  fill_vector_from_map(u_index_map(), u_i_map, nu(), u_i);
  return initialize_control_values(stage, u_i);
}
int AcadosSolver::initialize_control_values(ValueVector & u_i)
{
  int status = 0;
  // From 0 to N-1 as there is no control at terminal node!
  for (unsigned int stage = 0; stage < N(); stage++) {
    status += initialize_control_values(stage, u_i);
  }
  return status;
}
int AcadosSolver::initialize_control_values(ValueMap const & u_i_map)
{
  if (!is_values_map_complete(u_index_map(), u_i_map)) {
    return 1;
  }
  std::vector<double> u_i;
  u_i.reserve(nu());
  fill_vector_from_map(u_index_map(), u_i_map, nu(), u_i);
  return initialize_control_values(u_i);
}

// ------------------------------------------
// Runtime parameters
// ------------------------------------------

int AcadosSolver::set_runtime_parameters(unsigned int stage, ValueVector & p_i)
{
  if (p_i.size() != np()) {
    std::cerr << "WARNING: Failed to set p_" << stage << "!"
              << "A vector of length " << np() << " is expected (" << p_i.size() << " provided)." <<
      std::endl;
    return 1;
  }
  if (stage > N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::set_runtime_parameters()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  return internal_update_params(stage, p_i.data(), np());
}

int AcadosSolver::set_runtime_parameters(unsigned int stage, ValueMap const & p_i_map)
{
  if (!is_values_map_complete(p_index_map(), p_i_map)) {
    return 1;
  }
  std::vector<double> p_i;
  p_i.reserve(np());
  fill_vector_from_map(p_index_map(), p_i_map, np(), p_i);
  return set_runtime_parameters(stage, p_i);
}

int AcadosSolver::set_runtime_parameters(ValueVector & p_i)
{
  // From 0 to N to also set terminal node parameters!
  int status = 0;
  for (unsigned int stage = 0; stage <= N(); stage++) {
    status += set_runtime_parameters(stage, p_i);
  }
  return 0;
}

int AcadosSolver::set_runtime_parameters(ValueMap const & p_i_map)
{
  if (!is_values_map_complete(p_index_map(), p_i_map)) {
    return 1;
  }
  std::vector<double> p_i;
  p_i.reserve(np());
  fill_vector_from_map(p_index_map(), p_i_map, np(), p_i);
  return set_runtime_parameters(p_i);
}


//####################################################
//                     GETTERS
//####################################################

ValueVector AcadosSolver::get_state_values(unsigned int stage)
{
  if (stage > N()) {
    std::string err_msg = "Error in 'AcadosSolver::get_state_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  std::vector<double> x_i(nx(), 0.0);
  ocp_nlp_out_get(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "x", x_i.data());
  return x_i;
}

ValueMap AcadosSolver::get_state_values_as_map(unsigned int stage)
{
  return create_map_from_values(x_index_map(), get_state_values(stage));
}

ValueVector AcadosSolver::get_algebraic_state_values(unsigned int stage)
{
  if (stage >= N()) {
    std::string err_msg =
      "Error in 'AcadosSolver::get_algebraic_state_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  std::vector<double> z_i(nz(), 0.0);
  ocp_nlp_out_get(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "z", z_i.data());
  return z_i;
}
ValueMap AcadosSolver::get_algebraic_state_values_as_map(unsigned int stage)
{
  return create_map_from_values(z_index_map(), get_algebraic_state_values(stage));
}

ValueVector AcadosSolver::get_control_values(unsigned int stage)
{
  if (stage >= N()) {
    std::string err_msg = "Error in 'AcadosSolver::get_control_values()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  std::vector<double> u_i(nu(), 0.0);
  ocp_nlp_out_get(get_nlp_config(), get_nlp_dims(), get_nlp_out(), stage, "u", u_i.data());
  return u_i;
}
ValueMap AcadosSolver::get_control_values_as_map(unsigned int stage)
{
  return create_map_from_values(u_index_map(), get_control_values(stage));
}

ValueVector AcadosSolver::get_parameter_values(unsigned int stage)
{
  if (stage > N()) {
    std::string err_msg = "Error in 'AcadosSolver::get_parameters()': Invalid stage request!";
    throw std::range_error(err_msg);
  }
  std::vector<double> p_i(np(), 0.0);
  ocp_nlp_in_get(get_nlp_config(), get_nlp_dims(), get_nlp_in(), stage, "p", p_i.data());
  return p_i;
}

ValueMap AcadosSolver::get_parameter_values_as_map(unsigned int stage)
{
  return create_map_from_values(p_index_map(), get_parameter_values(stage));
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

bool AcadosSolver::is_map_size_consistent(IndexMap const & map, unsigned int expected_total_values)
{
  unsigned int total_values = 0;
  for (const auto & [key, value] : map) {
    total_values += value.size();
  }
  return total_values == expected_total_values;
}

bool AcadosSolver::is_map_size_consistent(ValueMap const & map, unsigned int expected_total_values)
{
  unsigned int total_values = 0;
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
      std::cerr << "key '" << key << "' not found!" << std::endl;
      all_ok = false;
      break;
    }
    all_ok &= (indexes.size() == values_map.at(key).size());
    if (!all_ok) {
      std::cerr << "key '" << key << "' has values of incorrect size!" << std::endl;
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
  ValueVector & values)
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
ValueVector AcadosSolver::value_vector_from_map(
  IndexMap const & index_map,
  ValueMap const & values_map,
  unsigned int vector_size)
{
  // Create container
  std::vector<double> value_vector(vector_size);

  // Fill the vector and return
  fill_vector_from_map(index_map, values_map, vector_size, value_vector);
  return value_vector;
}


ValueMap AcadosSolver::create_map_from_values(
  IndexMap const & index_map,
  ValueVector const & values)
{
  if (!is_map_size_consistent(index_map, values.size())) {
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

void AcadosSolver::fill_map_from_values(
  IndexMap const & index_map,
  ValueVector const & values,
  ValueMap & value_map)
{
  // Check the size of the map is consistent with the values vector
  // Note: this is not strictly necessary, but it is a good practice to ensure consistency
  //       between the index map and the values vector.
  if (!is_map_size_consistent(index_map, values.size())) {
    throw std::invalid_argument("Inconsistent data provided to 'fill_map_from_values()'!");
  }

  // Fill the map with new values
  // Note: this will overwrite existing keys in value_map
  //       that are also in index_map.
  for (const auto & [key, indexes] : index_map) {
    std::vector<double> tmp_data(indexes.size(), 0.0);
    for (unsigned int i = 0; i < indexes.size(); i++) {
      tmp_data[i] = values[indexes[i]];
    }
    value_map[key] = tmp_data;
  }
}

//####################################################
//                  DIMENSIONS
//####################################################

const AcadosSolver::Dimensions & AcadosSolver::dims() const
{
  return _dims;
}
unsigned int AcadosSolver::nx() const
{
  // return *(get_nlp_dims()->nx);
  return dims().nx;
}
unsigned int AcadosSolver::nz() const
{
  // return *(get_nlp_dims()->nz);
  return dims().nz;
}
unsigned int AcadosSolver::np() const
{
  // return static_cast<int>(get_nlp_np());
  return dims().np;
}
unsigned int AcadosSolver::nu() const
{
  // return *(get_nlp_dims()->nu);
  return dims().nu;
}
unsigned int AcadosSolver::N() const
{
  return get_nlp_dims()->N;
}
double AcadosSolver::Ts() const
{
  return _Ts;
}
ValueVector AcadosSolver::sampling_intervals() const
{
  std::vector<double> sampling_intervals_vect;
  sampling_intervals_vect.assign(get_nlp_in()->Ts, get_nlp_in()->Ts + N());
  return sampling_intervals_vect;
}

}  // namespace acados
