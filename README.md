# BSL-Plotter

## Environment
- Ubuntu 20.04
- docker, docker-compose

[NOTE]
This repository DOES NOT support NVIDIA graphic driver.

## Installation
### Hardware
W.I.P

### Software
1. Clone this repository
    ```
    git clone --recursive https://github.com/kim-xps12/bsl-plotter.git
    ```

1. Launch docker container
    ```
    cd bsl-plotter/docker_ros
    docker-compose up -d
    ```

1. Attach container
    ```
    docker attach mynoetic 
    ```

1. Check GUI
    ```
    xeyes
    ```

1. Check OpenGL
    ```
    glxgears
    ```

1. Enjoy your robotics!

## ROS 
You need to operate inside a docker container (*mynoetic*).

1. Go workspace
    ```
    cd catkin_ws
    ```
1. Build workspace and load settings
    ```
    catkin build
    source devel/setup.bash
    ```

1. Launch gazebo
    ```
    roslaunch bsl_plotter2_description gazebo.launch
    ```