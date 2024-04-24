# Copyright 2023 ICUBE Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Thibault Poignonec (tpoignonec@unistra.fr)

# set(ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR ...) in parent CMakeLists!

# Add header and source files for the plugin
list(APPEND
    SOLVER_PLUGINS_SOURCES
    ${ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR}/mock_acados_solver/mock_acados_solver.hpp
)
list(APPEND
    SOLVER_PLUGINS_SOURCES
    ${ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR}/mock_acados_solver/mock_acados_solver.cpp
)

# Add .so generated C-code
list(APPEND
    SOLVER_PLUGINS_PRE_GENERATED_LIBS
    ${ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR}/mock_acados_solver/generated_c_code/libacados_ocp_solver_mock_acados_solver.so
)

# Add plugin.xml
list(APPEND
    SOLVER_PLUGINS_EXPORT_XML_FILES
    ${ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR}/mock_acados_solver/export_plugin.xml
)