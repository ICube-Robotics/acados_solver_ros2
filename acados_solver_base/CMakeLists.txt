cmake_minimum_required(VERSION 3.8)
project(acados_solver_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(acados_vendor_ros2 REQUIRED)


add_library(${PROJECT_NAME}
  # Base class
  src/acados_solver.cpp
  # Base class (details)
  src/acados_solver_utils.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
    acados_vendor_ros2
    Eigen3
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "ACADOS_SOLVERS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES ${PRE_GENERATED_ACADOS_LIBS}
  DESTINATION lib/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  set(MOCK_SOLVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test/mock_acados_solver)
  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
    # Exclude auto generated c-code and CMakeLists
    ${MOCK_SOLVER_DIR}/**
  )
  ament_lint_auto_find_test_dependencies()

  # Test solver interface
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(
    test_acados_solver_base
    test/mock_acados_solver/mock_acados_solver.cpp
    test/test_solver.cpp
    test/test_static_functions.cpp
  )
  target_include_directories(test_acados_solver_base PUBLIC include test)
  # Link generated C-code
  target_link_libraries(test_acados_solver_base
    ${PROJECT_NAME}
    ${MOCK_SOLVER_DIR}/generated_c_code/libacados_ocp_solver_mock_acados_solver.so
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  acados_solver_base
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  acados_vendor_ros2
  eigen3_cmake_module
  Eigen3
)

ament_package()
