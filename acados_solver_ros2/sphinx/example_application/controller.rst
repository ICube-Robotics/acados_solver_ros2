ROS2 controller
============================

In the previous section, we used Acados to define an optimal control problem (OCP) and solver with Acados.

In this section, we will show how to generate a solver for C++ ROS2 controllers and how to use it.

Export the solver plugin
-------------------------

To use the Acados solver in a C++ ROS2 controller, we first need to export the solver as a `pluginlib` plugin.
The Python module `acados_solver_plugins` provides a class `SolverPluginGenerator` to generate the plugin:

.. autoclass:: acados_solver_plugins.SolverPluginGenerator
    :noindex:
    :members:
    :undoc-members:
    :private-members:
    :exclude-members:

Note that the `generate_solver_plugin` method of the `SolverPluginGenerator` class takes semantic maps as arguments, mapping variable names to their indices in the state, control, and parameter vectors.

For the RRBot example, the export script is as follows:

.. code-block:: python

    model = AcadosModel()
    ...  # define the model (see previous section)
    ocp = AcadosOcp()
    ...  # define the OCP (see previous section)

    # define the semantic map of the OCP variables
    x_index_map = {
        'q': [0, 1],
        'q_dot': [2, 3],
    }
    z_index_map = {}  # N.B., actually not empty in demo code
    p_index_map = {
        'l0': [0],
        'l1': [1],
        'l2': [2],
        'm1': [3],
        'm2': [4],
        'p_ref': [5, 6],
        'p_dot_ref': [7, 8],
        'Q_pos_diag': [9, 10],
        'Q_vel_diag': [11, 12],
        'R_diag': [13, 14],
    }
    u_index_map = {
        'tau': [0, 1],
    }


    # export the solver plugin
    from acados_solver_plugins import SolverPluginGenerator

    solver_plugin_generator = SolverPluginGenerator()

    solver_plugin_generator.generate_solver_plugin(
        ocp=ocp,
        plugin_class_name='RrbotCartesianTracking',
        solver_description='Acados solver plugin to track a cartesian trajectory with the RRBot planar robot',  # noqa: E501
        x_index_map=x_index_map,
        z_index_map=z_index_map,
        p_index_map=p_index_map,
        u_index_map=u_index_map,
    )

The full code can be found in the `export_acados_solver_plugin.py` file in the `example_acados_controller/script` directory of the `acados_solver_ros2_examples` repository.

Finally, to export the plugin, simply run the script with Python and re-build the workspace:

.. code-block:: bash

    source install/setup.bash
    python3 ./<relative_path_to_script>.py
    colcon build


Use the plugin in a C++ controller
-----------------------------------


Notes and recommandations
--------------------------

- If not provided, the semantic map is constructed from CasADI variable names.

- By default, the plugin sources are exported to the src directory of the `acados_solver_plugins` package.
  This can be changed by setting the `plugin_sources_dir` argument of the `generate_solver_plugin` method.

- If the Acados solver C library is generated for the first time, the user will be prompted to install some Acados ecternal dependencies.
  This can be avoided by running the script as follows:

    .. code-block:: bash

        echo "y" | python3 ./<relative_path_to_script>.py
