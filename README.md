# acados_solver_ros2

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![CI](https://github.com/tpoignonec/acados_solver_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/tpoignonec/acados_solver_ros2/actions/workflows/ci.yml)
[![Build tests (iron)](../../actions/workflows/ci-iron.yaml/badge.svg?branch=main)](../../actions/workflows/ci-iron.yaml?query=branch:main)
[![Build tests (rolling)](../../actions/workflows/ci-rolling.yaml/badge.svg?branch=main)](../../actions/workflows/ci-rolling.yaml?query=branch:main)

## About ##

Helper packages to use the Acados solver for non-linear optimization in ROS2 applications.
The goal is to prototype NMPC solvers in Python and to seamlessly use them in C++ ROS2 applications (i.e., `ros2-control` controllers).

To date, the stack includes:
  - `acados_solver_base`: a wrapper C++ class for Acados solvers;
  - `acados_solver_plugins`: a templated interface between the wrapper and Acados auto-generated C-code. A minimalistic Python library provides simple generation of C++ solver plugins from Python Acados models;
  - `acados_solver_plugins_example`: a package to be used as a demo and as a template when starting a project using the acados solvers.


**For more information, please check the [documentation](https://tpoignonec.github.io/acados_solver_ros2/).**

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Thibault Poignonec:__ [tpoignonec@unistra.fr](mailto:tpoignonec@unistra.fr), @github: [tpoignonec](https://github.com/tpoignonec)
