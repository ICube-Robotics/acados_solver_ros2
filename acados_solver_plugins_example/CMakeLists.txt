cmake_minimum_required(VERSION 3.10)
project(acados_solver_plugins_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(acados_solver_base REQUIRED)
find_package(pluginlib REQUIRED)

#-----------------------------------------------------
#   Populate cmake variables from available plugins
#-----------------------------------------------------
macro(SUBDIRLIST RESULT SEARCH_DIR)
  file(GLOB CHILDREN RELATIVE ${SEARCH_DIR} ${SEARCH_DIR}/*)
  set(DIRLIST "")
  foreach(CHILD ${CHILDREN})
    if(IS_DIRECTORY ${SEARCH_DIR}/${CHILD})
      list(APPEND DIRLIST ${CHILD})
    endif()
  endforeach()
  set(${RESULT} ${DIRLIST})
endmacro()

set(ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/plugins)
message(STATUS "Looking for plugins in '${ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR}'")

subdirlist(PLUGIN_DIR_LIST ${ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR})

# Transform to absulate paths
list(TRANSFORM PLUGIN_DIR_LIST PREPEND ${ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR}/)

set(SOLVER_PLUGINS_SOURCES "")
set(SOLVER_PLUGINS_PRE_GENERATED_LIBS "")
set(SOLVER_PLUGINS_EXPORT_XML_FILES "")

foreach(PLUGIN_DIR ${PLUGIN_DIR_LIST})
  message(STATUS "Adding plugin found in '${PLUGIN_DIR}'...")
  include(${PLUGIN_DIR}/add_pluging_to_build.cmake)
endforeach()

message(STATUS "PLUGIN_DIR_LIST : '${PLUGIN_DIR_LIST}'")
message(STATUS "SOLVER_PLUGINS_SOURCES : '${SOLVER_PLUGINS_SOURCES}'")
message(STATUS "SOLVER_PLUGINS_PRE_GENERATED_LIBS : '${SOLVER_PLUGINS_PRE_GENERATED_LIBS}'")
message(STATUS "SOLVER_PLUGINS_EXPORT_XML_FILES : '${SOLVER_PLUGINS_EXPORT_XML_FILES}'")


#-----------------------------------------------------
#   Setup CMAKE config regeneration from flag
#-----------------------------------------------------

set( FLAG_FILE_REBUILD ${CMAKE_CURRENT_SOURCE_DIR}/src/plugins/.flag_regenerate_cmake )
execute_process( COMMAND touch ${FLAG_FILE_REBUILD} )
add_custom_target(regenerate_cmake ALL)
set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${FLAG_FILE_REBUILD}
)

message(
  "${PROJECT_NAME}: the cmake config will be regenerated if the flag 'src/plugins/.flag_regenerate_cmake' is modified/deleted."
)

#-----------------------------------------------------
#   Export cpp library
#-----------------------------------------------------
if(SOLVER_PLUGINS_SOURCES)
  add_library(${PROJECT_NAME} ${SOLVER_PLUGINS_SOURCES})
  target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories(${PROJECT_NAME} PRIVATE ${PLUGIN_DIR_LIST})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${SOLVER_PLUGINS_PRE_GENERATED_LIBS})
  ament_target_dependencies(${PROJECT_NAME} PUBLIC acados_solver_base pluginlib)

  install(
    DIRECTORY include/
    DESTINATION include
  )
  install(
    FILES ${SOLVER_PLUGINS_PRE_GENERATED_LIBS}
    DESTINATION lib/
  )
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

# Install test code
add_executable(test_mock_plugin src/test_mock_plugin.cpp)
target_compile_features(test_mock_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(test_mock_plugin PUBLIC pluginlib acados_solver_base)


#-----------------------------------------------------
#   Tests
#-----------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#-----------------------------------------------------
#   Install
#-----------------------------------------------------

# Install test
install(TARGETS test_mock_plugin
  DESTINATION lib/${PROJECT_NAME})

# Install plugins
if(SOLVER_PLUGINS_SOURCES)
  foreach(PLUGIN_DESCRIPTION_XML_FILE ${SOLVER_PLUGINS_EXPORT_XML_FILES})
    message(STATUS "Exporting plugin from descr file'${PLUGIN_DESCRIPTION_XML_FILE}'")
    file(RELATIVE_PATH REL_PATH_TO_XML_FILE "${CMAKE_CURRENT_SOURCE_DIR}" "${PLUGIN_DESCRIPTION_XML_FILE}")
    pluginlib_export_plugin_description_file(acados_solver_base ${REL_PATH_TO_XML_FILE})
  endforeach()

  ament_export_include_directories(
    include
  )
  ament_export_libraries(
    ${PROJECT_NAME}
  )
  ament_export_targets(
    export_${PROJECT_NAME}
  )
  ament_export_dependencies(
    acados_solver_base pluginlib
  )
endif()

ament_package()
