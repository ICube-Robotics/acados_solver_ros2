from acados_template import AcadosOcp, AcadosOcpSolver, ocp_get_default_cmake_builder
# import casadi as ca

from jinja2 import PackageLoader, Environment

import os
import re
from copy import deepcopy
from acados_solver_plugins.utils import ensure_dir_exists, delete_dir_recursively

_library_name = "acados_solver_plugins"

# Reformat strings from capital to underscore
_uppercase_part = re.compile('[A-Z][^A-Z]*')


def uppercase_to_underscore(name):
    result = ''
    for match in _uppercase_part.finditer(name):
        if match.span()[0] > 0:
            result += '_'
        result += match.group().lower()
    return result

# Custom filters for jinja2


def filter_uppercase_to_underscore(name):
    return uppercase_to_underscore(name)


def filter_curly_bracket_list(list_of_index):
    return '{%s}' % str(list_of_index).strip('[]')

# Plugin generator


class SolverPluginGenerator:
    def __init__(self, custom_export_path=None, library_name=None):
        # Setup paths
        if custom_export_path is not None:
            self.__plugin_export_path = deepcopy(custom_export_path)
        else:
            self.__plugin_export_path = os.environ.get(
                'ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR'
            )
        print('SolverPluginGenerator: \
               All generated files will be exported to "' + self.__plugin_export_path + '".')

        # Setup jinja2 environment
        self.jinga_env_loader = PackageLoader("acados_solver_plugins")
        self.jinja_env = Environment(loader=self.jinga_env_loader)
        self.jinja_env.filters['curly_bracket_list'] = filter_curly_bracket_list
        self.jinja_env.filters['uppercase_to_underscore'] = filter_uppercase_to_underscore

        # Jinja2 templates
        self.__solver_pluging_cpp_template = "acados_solver_plugin.cpp.j2"
        self.__solver_pluging_hpp_template = "acados_solver_plugin.hpp.j2"
        self.__solver_pluging_cmake_template = "add_pluging_to_build.cmake.j2"
        self.__solver_pluging_export_xml_template = "export_plugin.xml.j2"

        # Misc. options
        self.__library_name = deepcopy(library_name)
        if library_name is None:
            self.__library_name = _library_name
        self.__clean_c_build_folder = True
        self.__generate_libplugin_export = True
        self.__default_plugin_description = "Acados solver plugin based on 'acados_solver_base'."

    def generate_file_from_template(
            self,
            template_filename,
            export_filename,
            **template_render_args):
        template = self.jinja_env.get_template(template_filename)
        try:
            with open(export_filename, "w") as f:
                f.write(template.render(**template_render_args))
        except Exception as e:
            print('Failed to generate the file "%s" from the template "%s". Exception: %s' % (
                export_filename, template_filename, e))
            raise

    def generate_solver_plugin(self,
                               acados_ocp: AcadosOcp,
                               plugin_class_name: str,
                               solver_description: str = None,
                               x_index_map: dict = None,
                               z_index_map: dict = None,
                               p_index_map: dict = None,
                               u_index_map: dict = None) -> int:
        # Make sure dir exist and clean if it does
        path_plugin_dir = os.path.join(
            self.__plugin_export_path, uppercase_to_underscore(plugin_class_name))
        print('Cleaning the target directory "%s"!' % (path_plugin_dir))
        if os.path.exists(path_plugin_dir):
            delete_dir_recursively(path_plugin_dir)
        else:
            ensure_dir_exists(path_plugin_dir)

        # Rename acados model and collect relevant information
        acados_ocp.model.name = uppercase_to_underscore(plugin_class_name)
        acados_ocp.code_export_directory = os.path.join(
            path_plugin_dir, "generated_c_code")
        acados_ocp_json_filename = os.path.join(
            path_plugin_dir, 'acados_ocp_' + acados_ocp.model.name + '.json')

        # Generate C-code
        print('Start generating/building the acados c-code for plugin "%s"!' %
              ("acados::"+plugin_class_name))
        ocp_cmake_builder = ocp_get_default_cmake_builder()
        ocp_solver = AcadosOcpSolver(
            acados_ocp,
            json_file=acados_ocp_json_filename,
            cmake_builder=ocp_cmake_builder
        )
        # Generate render args
        if solver_description is None:
            solver_description = self.__default_plugin_description
        template_render_args = {
            'solver_c_prefix': uppercase_to_underscore(plugin_class_name),
            'plugin_class_name': plugin_class_name,
            'solver_description': solver_description,
            'x_index_map': x_index_map,
            'z_index_map': z_index_map,
            'p_index_map': p_index_map,
            'u_index_map': u_index_map,
            'library_name': self.__library_name,
            'export_plugin': self.__generate_libplugin_export,
        }
        # Generate c++ wrapper from template
        cpp_impl_file = os.path.join(
            path_plugin_dir, ""+uppercase_to_underscore(plugin_class_name)+".cpp")
        hpp_impl_file = os.path.join(
            path_plugin_dir, ""+uppercase_to_underscore(plugin_class_name)+".hpp")
        print('Generating "%s" from template...' % (cpp_impl_file))
        self.generate_file_from_template(
            self.__solver_pluging_cpp_template, cpp_impl_file, **template_render_args)
        print('Generating "%s" from template...' % (hpp_impl_file))
        self.generate_file_from_template(
            self.__solver_pluging_hpp_template, hpp_impl_file, **template_render_args)

        # Generate plugin export and cmake utils
        cmake_file = os.path.join(
            path_plugin_dir, "add_pluging_to_build.cmake")
        plugin_xml_file = os.path.join(path_plugin_dir, "export_plugin.xml")
        print('Generating "%s" from template...' % (cmake_file))
        self.generate_file_from_template(
            self.__solver_pluging_cmake_template, cmake_file, **template_render_args)
        print('Generating "%s" from template...' % (plugin_xml_file))
        self.generate_file_from_template(
            self.__solver_pluging_export_xml_template, plugin_xml_file, **template_render_args)

        # Clean build files
        if self.__clean_c_build_folder:
            # Delete "custom_export_path/c_generated_code/build" folder and contents
            legacy_build_folder = os.path.join(
                path_plugin_dir, "generated_c_code/build/")
            if os.path.exists(legacy_build_folder):
                delete_dir_recursively(
                    legacy_build_folder, keep_empty_folder=False)
            # Rename "custom_export_path/c_generated_code/CMakeLists.txt"
            os.rename(
                os.path.join(path_plugin_dir,
                             "generated_c_code/CMakeLists.txt"),
                os.path.join(path_plugin_dir,
                             "generated_c_code/legacy_CMakeLists.txt")
            )
        # return the python wrapper of the acados ocp in case of
        return ocp_solver
