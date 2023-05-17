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
### Use terminator instead of tmux
```bash
# Outside the docker container
cd bsl_plotter
sudo apt update
sudo apt install terminator
# If using customized settings
mkdir -p ~/.config/terminator/
cp terminator/config ~/.config/terminator/config
# Launch terminator(Run after `docker-compose up -d`)
cd docker_ros/
docker-compose up -d
cd ..
terminator
```
