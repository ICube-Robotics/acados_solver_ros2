cmake_minimum_required(VERSION 3.8)
project(acados_solvers_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(acados_solver_plugins REQUIRED)

# Add node
add_executable(solver_test_node src/solver_test_node.cpp)
target_include_directories(solver_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(solver_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(solver_test_node PUBLIC acados_solver_plugins)

# Install
install(TARGETS solver_test_node
  DESTINATION lib/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  # None for now...
  # TODO: actual tests in acados_solver_base and acados_solver_plugins
endif()

ament_package()
