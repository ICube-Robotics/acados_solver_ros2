cmake_minimum_required(VERSION 3.8)
project(acados_solvers_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(acados_solvers REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(solver_test_node src/solver_test_node.cpp)
target_include_directories(solver_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(solver_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(solver_test_node PUBLIC acados_solvers)


install(TARGETS solver_test_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # None for now...
  # TODO: actual tests in acados_solvers and acados_solvers_plugins
endif()

ament_package()
