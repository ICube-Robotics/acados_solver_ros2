# acados_solver_ros2

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

[![CI (humble)](https://github.com/ICube-Robotics/acados_solver_ros2/actions/workflows/ci-humble.yml/badge.svg?branch=main)](https://github.com/ICube-Robotics/acados_solver_ros2/actions/workflows/ci-humble.yml)
[![CI (jazzy)](https://github.com/ICube-Robotics/acados_solver_ros2/actions/workflows/ci-jazzy.yaml/badge.svg?branch=main)](https://github.com/ICube-Robotics/acados_solver_ros2/actions/workflows/ci-jazzy.yaml)
[![CI (rolling)](https://github.com/ICube-Robotics/acados_solver_ros2/actions/workflows/ci-rolling.yaml/badge.svg)](https://github.com/ICube-Robotics/acados_solver_ros2/actions/workflows/ci-rolling.yaml)

## About ##

Helper packages to use the Acados solver for non-linear optimization in ROS2 applications.
The goal is to prototype NMPC solvers in Python and to seamlessly use them in C++ ROS2 applications (i.e., `ros2-control` controllers).

To date, the stack includes:
  - `acados_solver_base`: a wrapper C++ class for Acados solvers;
  - `acados_solver_plugins`: a templated interface between the wrapper and Acados auto-generated C-code. A minimalistic Python library provides simple generation of C++ solver plugins from Python Acados models;
  - `acados_solver_plugins_example`: a package to be used as a demo and as a template when starting a project using the acados solvers.


**For more information, please check the [documentation](https://icube-robotics.github.io/acados_solver_ros2/).**

## Installation ##

```bash
source /opt/ros/humble/setup.bash

mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ICube-Robotics/acados_solver_ros2.git

# Core dependencies
vcs import . < acados_solver_ros2/acados_solver_ros2.repos
rosdep install --ignore-src --from-paths . -y -r

# Python retro-compatibility
pip install future-fstrings

# Build ros2 packages
cd ..
colcon build
source install/setup.bash
```

> [!NOTE]
> If you are using ros jazzy or later, you might have to use a virtual environment or the `PIP_BREAK_SYSTEM_PACKAGES` flag.
> The recommended way to set this option when using rosdep is to set the environment variable
> `PIP_BREAK_SYSTEM_PACKAGES=1` before calling `rosdep` or `pip install`.
>
> Otherwise, you might get the following error:
> ```bash
> ERROR: the following rosdeps failed to install
>   pip:
> rosdep installation of pip packages requires installing packages globally as root.
> When using Python >= 3.11, PEP 668 compliance requires you to allow pip to install alongside
> externally managed packages using the 'break-system-packages' option.
> ```

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Thibault Poignonec:__ [tpoignonec@unistra.fr](mailto:tpoignonec@unistra.fr), @github: [tpoignonec](https://github.com/ICube-Robotics)
