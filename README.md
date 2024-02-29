# BSL-Plotter

![3D model](./bsl-plotter.png)

## Environment
- Ubuntu 20.04
- docker, docker compose

[NOTE]
This repository DOES NOT support NVIDIA graphic driver.

## Installation
### Hardware
The BSL-Plotter parts are in the `bsl-plotter/stl` directory. Please refer to URDF model and above image for assembly. And, use 6mm BB-bullets-ball to assemble the thrust bearings.
(3D model for assembly will be uploaded soon!)

It is recommended to use a 3D printer to create the parts. The recommended printing conditions are as follows.
- Nozzle: 0.4mm
- Filament: ABS, PLA, PETG
- Support: enable
- Wall: 0.6~1.0mm

**BOM**
| Item | Model | Quat. | Link |
| --- | --- | --- | --- |
| Servo Motor    | STS3215, Feetech   | 3 pcs. | [akitsuki](https://akizukidenshi.com/catalog/g/gM-16312/) |
| Servo IF Board | FE-URT-1, Feetech  | 1 pc   | [akitsuki](https://akizukidenshi.com/catalog/g/gM-16295/) |
| Filament | PolyTerra PLA, Polymaker | 1 roll | [Amazon](https://amzn.to/4028WbJ) |
| Magnet | コクヨ マグネット 強力マグネットプレート 片面・粘着剤付き 6枚 耐荷重500g マク-S381 | 1 pack | [Amazon](https://amzn.to/3FkPehZ) |
| White board | トレー付大きなホワイトボード ４５×６０ｃｍ   | 1 pc | [DAISO](https://jp.daisonet.com/products/4549131460452?_pos=28&_sid=489c126bd&_ss=r) |
| Pen | ホワイトボードマーカー（消し付、細芯、黒・赤・青、３本） |  1 pack | [DAISO](https://jp.daisonet.com/products/4549892198038?_pos=15&_sid=5683c238f&_ss=r) |
| BB-Ball (for thrust bearing unit) | BB弾 | 1 pack | [DAISO](https://jp.daisonet.com/products/4549131354997) |
| Wire | 3 core cable | about 1 m | | 
| Bolt | M2 tapping |  | included STS3215 |
|      | M3-5mm  |  | included STS3215 |
|      | M3-10mm | 4 pcs. | |
|      | M3-15mm | 2 pcs. | |
|      | M3-25mm | 4 pcs. | |
|      | M3-30mm | 2 pcs. | |
|      | M3-35mm | 4 pcs. | |
|      | M3-40mm | 4 pcs. | |
| Nuts | M3 Hex  | 20 pcs. | |

**Servo ID Setting**
- Root Joint: 1
- Middle Joint: 2
- Hand Joint: 3

**Cable Extension**
Extend the cable included with STS3215 to double the length. You can use a connector or solder it.

### Software
1. Clone this repository
    ```
    git clone --recursive https://github.com/kim-xps12/bsl-plotter.git
    ```

1. Launch docker container
    ```
    cd bsl-plotter/docker_ros
    docker compose up -d
    ```

1. Execute a command in a running container
    ```
    docker compose exec mynoetic /bin/bash
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
It is required to be able to use multiple terminals using *tmux* or *terminator*. I recommend reading "How to use Terminator" in the Reference section.

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
    roslaunch bsl_plotter_description display.launch
    ```
1. Add new pane, and Launch IK solver
    ```
    roslaunch plotter_controller test_swing.launch 
    ```
1. Add new pane, Run servo driver
    ```
    rosrun plotter_controller feetech_driver.py
    ```

## G-code drawer

### How to draw

```
rosrun plotter_controller draw_gcode.py
```

launch file is comming soon...

**NOTICE!**
[Memory Usage Warning for Large G-code Files]
This implementation loads entire G-code files into memory, which can lead to high memory usage with large files. If handling large G-code files, be aware of your system's memory capacity to avoid potential issues. Consider splitting very large files to manage memory usage effectively.


## Reference
[How to use Terminator](terminator/how_to_use_terminator.md)
