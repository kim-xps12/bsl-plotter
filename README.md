# BSL-Plotter

## Environment
- Ubuntu 20.04
- docker, docker-compose

[NOTE]
This repository DOES NOT support NVIDIA graphic driver.

## Installation
### Hardware
The BSL-Plotter parts are in the `bsl-plotter/stl` directory. Please refer to URDF model for assembly.
(3D model for assembly will be uploaded soon!)

It is recommended to use a 3D printer to create the parts. The recommended printing conditions are as follows.
- Nozzle: 0.4mm
- Filament: ABS, PLA, PETG
- Support: enable
- Wall: 0.6~1.0mm

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

1. Execute a command in a running container
    ```
    docker-compose exec mynoetic /bin/bash
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

1. Launch rviz
    ```
    roslaunch bsl_plotter2_description gazebo.launch
    ```
1. Launch IK solver
    ```
    roslaunch plotter_controller solve_ik.launch
    ```
1. Run servo driver
    ```
    rosrun plotter_controller feetech_driver.py
    ```

## Reference
[How to use Terminator](terminator/how_to_use_terminator.md)
