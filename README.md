# acados_controllers_ros2

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![CI](https://github.com/tpoignonec/acados_controllers_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/tpoignonec/acados_controllers_ros2/actions/workflows/ci.yml)
[![Build tests (iron)](../../actions/workflows/ci-iron.yaml/badge.svg?branch=main)](../../actions/workflows/ci-iron.yaml?query=branch:main)
[![Build tests (rolling)](../../actions/workflows/ci-rolling.yaml/badge.svg?branch=main)](../../actions/workflows/ci-rolling.yaml?query=branch:main)
Ros2 controllers using Acados and ros2-control


## Getting started

1. Install the `ros2` distribution (current dev. based on `ros2 humble`). See the [documentation](https://docs.ros.org/en/humble/Installation.html) for installation steps.

2. Source the `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
3. Prepare the workspace and install dependencies
    ```shell
    sudo apt install python3-colcon-common-extensions
    cd ~/ros2_ws
    cd src
    git clone https://github.com/tpoignonec/acados_controllers_ros2.git
    vcs import . < acados_controllers_ros2/acados_controllers_ros2.repos
    rosdep install --ignore-src --from-paths . -y -r
    ```

4. Build stack and source
    ```shell
    cd ~/ros2_ws
    colcon build && colcon build
    source install/setup.bash
    ```
