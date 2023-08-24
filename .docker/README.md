
To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

```shell
$ docker build --tag acados_controllers_ros2:humble --file .docker/run/Dockerfile .
```

### Run with GUI
To run the docker image with GUI, use the [rocker tool](https://github.com/osrf/rocker):
```shell
$ sudo apt install python3-rocker
$ rocker --net=host --x11 acados_controllers_ros2:humble ros2 run sandbox test_urdf2casadi
```

### Run with bash
To interact with the environment, run docker using:
```shell
$ docker run -it acados_controllers_ros2:humble
```
and inside docker run:
```shell
$ cd ros2_dev/acados_controllers_ros2/
$ source install/local_setup.bash
$ ros2 run sandbox test_urdf2casadi
```
