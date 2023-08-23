cmake_minimum_required(VERSION 3.8)
project(acados_solvers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(acados_vendor_ros2 REQUIRED)

add_library(acados_solvers
  # Base class
  src/acados_solver.cpp
)
target_compile_features(acados_solvers PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(acados_solvers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(acados_solvers PUBLIC acados_vendor_ros2)
target_compile_definitions(acados_solvers PRIVATE "ACADOS_SOLVERS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES ${PRE_GENERATED_ACADOS_LIBS}
  DESTINATION lib/
)

install(
  TARGETS acados_solvers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  set(C_CODE_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test/mock_acados_solver/generated_c_code)
  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
    # Exclude auto generated c-code and CMakeLists
    ${C_CODE_TEST_DIR}/**
  )
  ament_lint_auto_find_test_dependencies()

  # Test solver interface
  ament_add_gmock(
    test_acados_solver_base
    test/mock_acados_solver/mock_acados_solver.cpp
    test/test_solver.cpp
    test/test_static_functions.cpp
  )
  target_include_directories(test_acados_solver_base PUBLIC include test)
  # Link generated C-code
  target_link_libraries(test_acados_solver_base
    acados_solvers
    ${C_CODE_TEST_DIR}/libacados_ocp_solver_mock_acados_solver.so
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  acados_solvers
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(acados_vendor_ros2)

ament_package()
