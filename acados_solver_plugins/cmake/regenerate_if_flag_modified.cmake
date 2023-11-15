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
# Author: Thibault Poignonec (thibault.poignonec@gmail.com)

# Part of the 'acados_controller_ros2' package

function(regenerate_if_flag_modified)
    set( FLAG_FILE_REBUILD ${CMAKE_CURRENT_SOURCE_DIR}/src/plugins/.flag_regenerate_cmake )
    execute_process( COMMAND touch ${FLAG_FILE_REBUILD} )
    add_custom_target(regenerate_cmake ALL)
    set_property(
        DIRECTORY
        APPEND
        PROPERTY CMAKE_CONFIGURE_DEPENDS
        ${FLAG_FILE_REBUILD}
    )
endfunction()
