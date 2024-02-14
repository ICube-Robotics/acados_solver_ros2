Getting started
===============================

Installation
****************

**Required setup : Ubuntu 22.04 LTS**

1. Install the ROS2 distribution (current dev. based on ``ros2 humble``).
See the `official documentation <https://docs.ros.org/en/humble/Installation.html>`_ for ROS2 installation steps.

2. Source the ROS2 environment:

.. code-block:: bash

    source /opt/ros/humble/setup.bash

3. Prepare the workspace and install dependencies

.. code-block:: bash

    sudo apt install python3-colcon-common-extensions
    cd ~/ros2_ws
    cd src
    git clone https://github.com/tpoignonec/acados_controllers_ros2.git
    vcs import . < acados_controllers_ros2/acados_controllers_ros2.repos
    rosdep install --ignore-src --from-paths . -y -r

4. Build stack and source

.. code-block:: bash

    cd ~/ros2_ws
    colcon build && colcon build
    source install/setup.bash

Example test code
******************

At this point, you should be able to export the example solver plugin ``acados::MockAcadosSolver``.

First, export the Acados solver plugin:

.. code-block:: bash

    cd ~/ros2_ws
    source install/setup.bash

    cd src/acados_controllers_ros2/acados_solver_plugins_example/script
    python3 -m demo_export_plugin_to_custom_pkg

    # Note: if used for the first time, Acados will ask you to install "Tera renderer"
    #   -> press "y" to accept


Then build the newly generated plugin and run the example program that solves a NMPC problem for a pendulum system:


.. code-block:: bash

    cd ~/ros2_ws
    colcon build
    source install/setup.bash

    ros2 run acados_solver_plugins_example test_mock_plugin
