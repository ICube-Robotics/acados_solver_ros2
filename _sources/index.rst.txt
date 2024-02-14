Acados controllers ROS2 Stack
=============================

Helper packages to use the Acados solver for non-linear optimization in ROS2 applications.

The goal is to prototype NMPC solvers in Python and to seamlessly use them in C++ ROS2 applications (i.e., ``ros2-control`` controllers).

To date, the stack includes:
  * ``acados_solver_base``: a wrapper C++ class for Acados solvers;
  * ``acados_solver_plugins``: a templated interface between the wrapper and Acados auto-generated C-code.
    A minimalistic Python library provides simple generation of C++ solver plugins from Python Acados models;
  * ``acados_solver_plugins_example``: a package to be used as a demo and as a template when starting a project using the acados solvers.

**Project GitHub repository:** `acados_solver_ros2 <https://github.com/tpoignonec/acados_solver_ros2>`_

.. toctree::
  :maxdepth: 1
  :caption: Quickstart
  :glob:

  quickstart/installation
  quickstart/usage
  quickstart/generate_new_solvers
  quickstart/write_controllers

.. toctree::
  :maxdepth: 1
  :caption: Python modules
  :glob:

  developer_guide/solver_plugins_python_API

.. toctree::
  :maxdepth: 1
  :caption: Extra documentation
  :glob:

  developer_guide/API
