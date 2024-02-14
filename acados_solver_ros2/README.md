### Acados solver utils for ROS2 applications

## Meta-package `acados_solver_ros2`

This is a meta-package Acados ROS2 controllers stack.

## How to build the documentation locally?

1) Make sure sphinx and doxygen are installed
```bash
source /opt/ros/<distro>/setup.bash
sudo apt install python3-sphinx doxygen doxygen-gui doxygen-doc
```

2) Install the necessary sphinx extensions
```bash
cd acados_solver_ros2/acados_solver_ros2
pip install -r requirements.txt
```

3) Build the documentation
```bash
sphinx-build -b html sphinx sphinx/_build
# Or to force a full re-build of the docs
sphinx-build -E -a -b html sphinx sphinx/_build
```

4) Open local documentation
```bash
ln -s sphinx/_build/index.html documentation.html  # create simlink
firefox documentation.html
```
