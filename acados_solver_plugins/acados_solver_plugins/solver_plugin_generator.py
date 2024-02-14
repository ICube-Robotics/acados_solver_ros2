# Copyright 2023 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author: Thibault Poignonec (tpoignonec@unistra.fr)

from copy import deepcopy
import os

from acados_template import AcadosOcp, AcadosOcpSolver
from acados_template import ocp_get_default_cmake_builder
# import casadi as ca
from jinja2 import Environment, PackageLoader  # noqa: I100

import numpy as np

from acados_solver_plugins.utils import delete_dir_recursively  # noqa: I100
from acados_solver_plugins.utils import ensure_dir_exists
from acados_solver_plugins.utils import uppercase_to_underscore

_library_name = 'acados_solver_plugins'


# Custom filters for jinja2


def filter_uppercase_to_underscore(name):
    return uppercase_to_underscore(name)


def filter_curly_bracket_list(list_of_index):
    if isinstance(list_of_index, list):
        return '{%s}' % str(list_of_index).strip('[]')
    elif isinstance(list_of_index, np.ndarray):
        return '{%s}' % str(list_of_index.reshape((-1,)).tolist()).strip('[]')
    else:
        raise Exception(
            'jinja2:filter_curly_bracket_list -> unsupported type!!!')


# Plugin generator

class SolverPluginGenerator:
    """
    Minimalist template-based code generation module to generate \
    `acados::AcadosSolver` derived objects.

    Relevant external API documentation : \
    `jinja2 <https://jinja.palletsprojects.com/en/3.0.x/api/>`_, \
    `acados_template <https://docs.acados.org/python_interface/index.html#>`_.
    """

    def __init__(
        self,
        custom_export_path: str = None,
        library_name: str = None
    ):
        """
        Solver plugin generator constructor.

        :param custom_export_path: Dir where the generated files will \
            be exported, defaults to None
        :type custom_export_path: str, optional
        :param library_name: Name of the library that the generated files \
            will be part of, defaults to None (in which case the library \
            is set to "acados_solver_plugins" internally)
        :type library_name: str, optional
        """
        # Setup paths
        if custom_export_path is not None:
            self.__plugin_export_path = deepcopy(custom_export_path)
        else:
            self.__plugin_export_path = os.environ.get(
                'ACADOS_SOLVER_PLUGINGS_GENERATION_EXPORT_DIR'
            )
        print('SolverPluginGenerator: \
               All generated files will be exported to "'
              + self.__plugin_export_path + '".')

        # Setup jinja2 environment

        #: Jinja environment loader initialized as a `jinja2.PackageLoader` \
        #: looking for templates in the installation folder of the module.
        self.jinga_env_loader = PackageLoader(_library_name)

        self.jinja_env = Environment(loader=self.jinga_env_loader)
        self.jinja_env.filters[
            'curly_bracket_list'] = filter_curly_bracket_list
        self.jinja_env.filters[
            'uppercase_to_underscore'] = filter_uppercase_to_underscore

        # Jinja2 templates
        self.__solver_pluging_cpp_template = 'acados_solver_plugin.cpp.j2'
        self.__solver_pluging_hpp_template = 'acados_solver_plugin.hpp.j2'
        self.__solver_pluging_cmake_template = 'add_pluging_to_build.cmake.j2'
        self.__solver_pluging_export_xml_template = 'export_plugin.xml.j2'

        # Misc. options
        self.__library_name = deepcopy(library_name)
        if library_name is None:
            self.__library_name = _library_name
        self.__clean_c_build_folder = True
        self.__generate_libplugin_export = True
        self.__default_plugin_description = \
            'Acados solver plugin based on "acados_solver_base".'
        self.__flag_regenerate_cmake = '.flag_regenerate_cmake'

    def _generate_file_from_template(
            self,
            template_filename,
            export_filename,
            **template_render_args):
        template = self.jinja_env.get_template(template_filename)
        try:
            with open(export_filename, 'w') as f:
                f.write(template.render(**template_render_args))
        except Exception as e:
            print(
                'Failed to generate the file '
                + '"{}" from the template "{}". Exception: {}'.format(
                    export_filename, template_filename, e))
            raise

    def generate_solver_plugin(self,
                               acados_ocp: AcadosOcp,
                               plugin_class_name: str,
                               solver_description: str = None,
                               x_index_map: dict = None,
                               z_index_map: dict = None,
                               p_index_map: dict = None,
                               u_index_map: dict = None) -> AcadosOcpSolver:
        """
        Generate the solver plugin from a python Acados OCP model.

        :param acados_ocp: Acados OCP model
        :type acados_ocp: AcadosOcp
        :param plugin_class_name:
            Class name for the generated plugin of type
            "acados::<plugin_class_name>"
        :type plugin_class_name: str
        :param solver_description:
            Short description for the exported plugin, defaults to None
        :type solver_description: str, optional
        :param x_index_map: Differential state index map, defaults to None
        :type x_index_map: dict, optional
        :param z_index_map: Algebraic state index map, defaults to None
        :type z_index_map: dict, optional
        :param p_index_map: Parameters index map, defaults to None
        :type p_index_map: dict, optional
        :param u_index_map: Parameters index map, defaults to None
        :type u_index_map: dict, optional
        :return: the acados OCP solver generated as a by-product
        :rtype: AcadosOcpSolver
        """
        # Make sure dir exist and clean if it does
        path_plugin_dir = os.path.join(
            self.__plugin_export_path,
            uppercase_to_underscore(plugin_class_name)
        )
        print('Cleaning the target directory "%s"!' % (path_plugin_dir))
        if os.path.exists(path_plugin_dir):
            delete_dir_recursively(path_plugin_dir)
        else:
            ensure_dir_exists(path_plugin_dir)

        # Rename acados model and collect relevant information
        acados_ocp.model.name = uppercase_to_underscore(plugin_class_name)
        acados_ocp.code_export_directory = os.path.join(
            path_plugin_dir, 'generated_c_code')
        acados_ocp_json_filename = os.path.join(
            path_plugin_dir, 'acados_ocp_' + acados_ocp.model.name + '.json')

        # Generate C-code
        print('Start generating/building the acados c-code for plugin "%s"!' %
              ('acados::'+plugin_class_name))
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
            path_plugin_dir,
            ''+uppercase_to_underscore(plugin_class_name)+'.cpp'
        )
        hpp_impl_file = os.path.join(
            path_plugin_dir,
            ''+uppercase_to_underscore(plugin_class_name)+'.hpp'
        )
        print('Generating "%s" from template...' % (cpp_impl_file))
        self._generate_file_from_template(
            self.__solver_pluging_cpp_template,
            cpp_impl_file,
            **template_render_args
        )
        print('Generating "%s" from template...' % (hpp_impl_file))
        self._generate_file_from_template(
            self.__solver_pluging_hpp_template,
            hpp_impl_file,
            **template_render_args
        )

        # Generate plugin export and cmake utils
        cmake_file = os.path.join(
            path_plugin_dir, 'add_pluging_to_build.cmake')
        plugin_xml_file = os.path.join(path_plugin_dir, 'export_plugin.xml')
        print('Generating "%s" from template...' % (cmake_file))
        self._generate_file_from_template(
            self.__solver_pluging_cmake_template,
            cmake_file,
            **template_render_args
        )
        print('Generating "%s" from template...' % (plugin_xml_file))
        self._generate_file_from_template(
            self.__solver_pluging_export_xml_template,
            plugin_xml_file,
            **template_render_args
        )

        # Clean build files
        if self.__clean_c_build_folder:
            # Delete "custom_export_path/c_generated_code/build" folder
            legacy_build_folder = os.path.join(
                path_plugin_dir, 'generated_c_code/build/')
            if os.path.exists(legacy_build_folder):
                delete_dir_recursively(
                    legacy_build_folder, keep_empty_folder=False)
            # Rename 'custom_export_path/c_generated_code/CMakeLists.txt'
            os.rename(
                os.path.join(path_plugin_dir,
                             'generated_c_code/CMakeLists.txt'),
                os.path.join(path_plugin_dir,
                             'generated_c_code/legacy_CMakeLists.txt')
            )

        # Trigger CMAKE build
        cmake_flag_name = os.path.join(
            self.__plugin_export_path,
            self.__flag_regenerate_cmake
        )
        if (os.path.exists(cmake_flag_name)):
            try:
                os.remove(cmake_flag_name)
                with open(cmake_flag_name, mode='a'):
                    pass
            except Exception as error:
                print(
                    'Warning, failed to reset the CMAKE flag ',
                    '"{cmake_flag_name}". Error: ',
                    error
                )
        else:
            print(
                f'Warning, the CMAKE flag "{cmake_flag_name}" does not exist.'
            )

        # return the python wrapper of the acados ocp in case of
        return ocp_solver
