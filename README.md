# Ranged IK ROS1

## Introduction

You can find an introduction and the citation information of RangedIK in the README of [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik) which is a submodule of this repo. It is recommended to look at [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik) before working with this wrapper.

## The RelaxedIK Family
[RangedIK](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik) extends [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core/tree/relaxed-ik) by leveraging the flexibility afforded by tolerances. Also, RangedIK is recently maintained and works with more recent rust versions.

[CollisionIK](https://github.com/uwgraphics/relaxed_ik_core/tree/collision-ik) extends [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core/tree/relaxed-ik) by avoiding collisions with static or dynamic obstacles in the environment.

We provide a series of wrappers for our tools to be used in various platform or software. 

||**ROS1**|**ROS2**|**WebAssembly**|**Coppeliasim**|**Mujoco**|  
|:------|:-----|:-----|:-----|:-----|:-----| 
|**RangedIK**|[link](https://github.com/uwgraphics/relaxed_ik_ros1/tree/ranged-ik)|[link](https://github.com/uwgraphics/relaxed_ik_ros2)|[link](https://github.com/yepw/relaxed-ik-web-demo/)|x|x|  
|**CollisionIK**|[link](https://github.com/uwgraphics/relaxed_ik_ros1/)|x|x|x|x|  
|**RelaxedIK**|[link](https://github.com/uwgraphics/relaxed_ik_ros1/)|x|x|[link](https://github.com/uwgraphics/relaxed_ik_coppeliasim)|[link](https://github.com/uwgraphics/relaxed_ik_mujoco)|  


## Dependencies

### Python Dependencies
**readchar**:
```bash
sudo pip install readchar
```
**Pyyaml**:
```bash
sudo pip install PyYaml
```

### Rust Dependencies
To use this wrapper, you will first need to install Rust. Please go to https://www.rust-lang.org/learn/get-started for more infomation.

## Getting Started

1. Compile relaxed_ik_core
2. Download mesh files for robots:

    UR5: https://github.com/ros-industrial/universal_robot

    Sawyer: https://github.com/RethinkRobotics/sawyer_robot

    Spot arm: https://github.com/heuristicus/spot_ros

    Baxter: https://github.com/RethinkRobotics/baxter_common  

3. ```catkin_make```
4. Start a demo with the Sawyer robot
    ```bash
    roslaunch relaxed_ik_ros1 demo.launch
    ```
    if you want use a different robot, pass the absolute path to the setting file as an argument
    ```bash
    roslaunch relaxed_ik_ros1 demo.launch setting_file_path:=<your absolute path to the setting file>
    ``` 

5a. To move the robot using keyboard, open a new terminal

```bash
rosrun relaxed_ik_ros1 keyboard_input.py
```

And then use the folloing commands to move the robot.
```bash
c - kill the controller controller script
w - move chain 1 along +X
x - move chain 1 along -X
a - move chain 1 along +Y
d - move chain 1 along -Y
q - move chain 1 along +Z
z - move chain 1 along -Z
1 - rotate chain 1 around +X
2 - rotate chain 1 around -X
3 - rotate chain 1 around +Y
4 - rotate chain 1 around -Y
5 - rotate chain 1 around +Z
6 rotate chain 1 around -Z
```
    
5b. To trace a predefined trajectory
```bash
rosrun relaxed_ik_ros1 line_tracing.py _tolerances:=[0,0,0,0,0,999]
```
which allows the end-effector to freely rotate allow the z axis. 
If you don't want to use tolerances, set the _tolerances arguments to all zeros. 
The script use ROS service by default. To use ROS topic, set parameter `_use_topic_not_service:=True` 