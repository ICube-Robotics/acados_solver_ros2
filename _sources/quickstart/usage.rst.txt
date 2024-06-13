Usage
=====

.. _fig-overview-software:
.. figure:: ../images/overview_soft.png
   :width: 800
   :align: center

   Overview of the prototyping steps.


1) Build Acados model
----------------------

Use `CasADI <https://web.casadi.org/>`_ to define the symbolic variables of the problem (i.e., state variables, control inputs, etc.) and the functions necessary to model dynamics of the system (i.e., explicit ODE or DAE).
Then use `acados_template <https://docs.acados.org/python_interface/index.html>`_ to instantiate an ``AcadosModel`` object that will be used to define the optimal control problem (OCP) in the next step.
We refer to the `Acados documentation <https://docs.acados.org/python_interface/index.html>`_ and `examples <https://github.com/acados/acados/tree/master/examples/acados_python>`_ for more details on how to build an Acados model.


2) Formulate the optimal control problem (OCP)
----------------------------------------------

Use the ``AcadosOcp`` class to define the optimal control problem (OCP) by specifying the `cost function <https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp_cost.AcadosOcpCost>`_, `constraints <https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp_constraints.AcadosOcpConstraints>`_, etc.
Then, set the solver options, especially the choice of QP solver (e.g., HPIPM, qpOASES, etc.) and integrator (e.g., ERK, IRK, etc.).


3) Gnererate the C++ plugin
----------------------------

Once the OCP is defined, use the ``acados_solver_plugins`` package to generate the solver C++ plugin.
The generation function takes as input the OCP, the name of the plugin class, the description of the solver, and the semantic map of the OCP variables.
The latter is used to provide a mapping between the OCP variables and the vector indices of the corresponding variables in the solver C++ interface (see the next step).

.. code-block:: python

    model = AcadosModel()
    ...  # define the model
    ocp = AcadosOcp()
    ...  # define the OCP

    # define the semantic map of the OCP variables
    # N.B., this example is for illustration purpose only
    x_index_map = {
        'a_state_variable': [0],
        'a_set_of_state_variables': [1, 2, 3],
        'another_state_variable': [4],
    }
    z_index_map = {}  # N.B., actually not empty in demo code
    p_index_map = {
        'a_parameter': [0],
        'another_parameter': [1],
        'a_set_of_parameters': [2, 3, 4],
    }
    u_index_map = {
        'some_control_variables': [0, 1],
    }

    # export the solver plugin
    from acados_solver_plugins import SolverPluginGenerator

    solver_plugin_generator = SolverPluginGenerator()

    solver_plugin_generator.generate_solver_plugin(
        ocp=ocp,
        plugin_class_name='name_of_the_plugin_class',
        solver_description='description_of_the_solver',
        x_index_map=x_index_map,
        z_index_map=z_index_map,
        p_index_map=p_index_map,
        u_index_map=u_index_map,
    )


4) Use the plugin in a C++ application
---------------------------------------

The generated plugin can be used in a C++ application to solve the OCP.
Start by loading the plugin and creating an instance of the solver (e.g., in the ``configure()`` method of your controller):

.. code-block:: cpp

    #include "acados_solver_base/acados_solver_base.hpp"
    #include "pluginlib/class_loader.hpp"

    // Acados solver pluginlib loader
    std::shared_ptr<pluginlib::ClassLoader<acados::AcadosSolver>> acados_solver_loader_;

    // Acados solver
    std::unique_ptr<acados::AcadosSolver> acados_solver_;

    // load the solver plugin
    acados_solver_loader_ =
        std::make_shared<pluginlib::ClassLoader<acados::AcadosSolver>>(
        "acados_solver_base", "acados::AcadosSolver");
    acados_solver_ = std::unique_ptr<acados::AcadosSolver>(
        acados_solver_loader_->createUnmanagedInstance(nmpc_plugin_name));

When you configure the solver at runtime, you can change the sampling period and the prediction horizon :

.. code-block:: cpp

    // configure the solver
    auto ret = acados_solver_->init(prediction_horizon, sampling_period_in_seconds);


Then simply use the solver to solve the OCP at each control loop iteration (e.g., in the ``update()`` method of your controller).
Some basic functionalities are wrapped in the ``acados::AcadosSolver`` class, such as :

- ``set_initial_state_values()``: set the initial state of the OCP
- ``set_runtime_parameters()``: set the parameters of the OCP
- ``set_control_bounds()``: set the control variables upper and lower bounds
- ``set_state_bounds()``: set the state variables upper and lower bounds
- ``solve()``: solve the OCP
- ``get_control_values()``: get the control values of the solution
- ``get_state_values()``: get the predicted state values at a given stage (i.e., time step of the prediction horizon)

The solver variables (i.e., state, control, etc.) can be accessed as ``std::vector<double>`` or using the semantic map of the OCP variables.
A set of helper functions is also provided to access higher-level functionalities of the solver, such as the cost function value.
Finally, the C interface can be used to access more advanced functionalities of the solver that are not wrapped in the ``AcadosSolver`` class.
To retrieve the C interface, use the methods ``get_nlp_out ()``, ``get_nlp_in ()``, ``get_nlp_opts ()``, etc.
