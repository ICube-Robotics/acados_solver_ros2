cmake_minimum_required(VERSION 3.8)
project(acados_controllers_ros2 NONE)
find_package(ament_cmake REQUIRED)


option(BUILD_DOCS
  "Build the doc (doxygen + sphinx)"
  OFF
)

if(${BUILD_DOCS})
  find_package(Python COMPONENTS Interpreter REQUIRED)

  if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
    set(pip_args)
  else()
    set(pip_args "--user")
  endif()

  execute_process(COMMAND ${Python_EXECUTABLE} -m pip install ${_pip_args} -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)

  set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/sphinx)
  set(SPHINX_BUILD ${CMAKE_CURRENT_SOURCE_DIR}/sphinx/_build)

  add_custom_target(Sphinx ALL
                    COMMAND
                    sphinx-build -b html
                    ${SPHINX_SOURCE} ${SPHINX_BUILD}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                    COMMENT "Generating documentation with Sphinx")
endif()

ament_package()
