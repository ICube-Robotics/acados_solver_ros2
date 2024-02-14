#!/usr/bin/env python3

import sys

from acados_solver_plugins import SolverPluginGenerator

from export_acados_ocp import export_pendulum_cart_acados_ocp


def main() -> int:
    # Create ocp
    acados_ocp = export_pendulum_cart_acados_ocp()

    # Define the index maps
    x_index_map = {
        'p': [0],
        'p_dot': [1],
        'theta': [2],
        'theta_dot': [3],
    }
    z_index_map = {}
    p_index_map = {
        'mass_cart': [0],
        'mass_ball': [1],
    }
    u_index_map = {
        'f': [0],
    }

    # Instantiate plugin generator
    solver_plugin_generator = SolverPluginGenerator()

    solver_plugin_generator.generate_solver_plugin(
        acados_ocp,
        plugin_class_name='MockAcadosSolver',
        solver_description='This is an example of Acados solver plugin. The underlying model is a pendulum on a cart.',  # noqa: E501
        x_index_map=x_index_map,
        z_index_map=z_index_map,
        p_index_map=p_index_map,
        u_index_map=u_index_map,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
