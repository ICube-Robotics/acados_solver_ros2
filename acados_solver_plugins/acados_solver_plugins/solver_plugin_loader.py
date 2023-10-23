# Copyright 2023 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author: Thibault Poignonec (tpoignonec@unistra.fr)

# from acados_template import AcadosOcp, AcadosOcpSolver
# from acados_template import  ocp_get_default_cmake_builder
# import casadi as ca

# from copy import deepcopy
# import os

# from acados_solver_plugins.utils import uppercase_to_underscore

# Plugin loader


class SolverPluginLoader:
    def __init__(
        self,
        custom_import_path: str = None,
        library_name: str = None
    ):
        """
        Solver plugin loader constructor.

        :param custom_import_path: Dir where the generated files were \
            exported by the generator, defaults to None
        :type custom_import_path: str, optional
        :param library_name: Name of the library that the generated files \
            are part of, defaults to None (in which case the library \
            is set to "acados_solver_plugins" internally)
        :type library_name: str, optional
        """
        pass

    def load_solver_plugin(self, solver_plugin_name: str):
        pass
