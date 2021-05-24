# Relaxed IK ROS1

## Introduction

You can find an introduction and the citation information of RelaxedIK in the README of [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core) which is a submodule of this repo. It is recommended to look at [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core) before working with this wrapper.

Relaxed IK ROS1 has the complete set of features available in the Relaxed IK family and it is also where [CollisionIK](https://arxiv.org/abs/2102.13187) and Pathwise CollisionIK (In development) reside. If you doesn't have strong preferences over any specific wrapper, you probably should consider this ROS1 wrapper as the first choice. A keyboard pose goal driver and an rviz viewer are provided for driving the robot's end-effector around in a simulated environment. 

## Relaxed IK Family

More information about Relaxed IK, CollisionIK, and all the wrappers could be found in this [documentation](https://uwgraphics.github.io/relaxed_ik_core/).

- [Relaxed IK (Deprecated)](https://github.com/uwgraphics/relaxed_ik/tree/dev)
- [Relaxed IK Core](https://github.com/uwgraphics/relaxed_ik_core)
- [Relaxed IK ROS1](https://github.com/uwgraphics/relaxed_ik_ros1)
- [Relaxed IK Unity](https://github.com/uwgraphics/relaxed_ik_unity)
- [Relaxed IK CoppeliaSim](https://github.com/uwgraphics/relaxed_ik_coppeliasim)
- [Relaxed IK Mujoco](https://github.com/uwgraphics/relaxed_ik_mujoco)

||**Relaxed IK (Deprecated)**|**Relaxed IK ROS1**|**Relaxed IK Unity**|**Relaxed IK Coppeliasim**|**Relaxed IK Mujoco**|  
|:------|:-----|:-----|:-----|:-----|:-----| 
|**Relaxed IK**|:o:|:o:|:o:|:o:|:o:|  
|**CollisionIK**|:x:|:o:|:x:|:x:|:x:|  

## Dependencies

### Python Dependencies
**readchar**:
```bash
sudo pip install readchar
```
**fcl collision library**: https://github.com/BerkeleyAutomation/python-fcl
```bash
sudo pip install python-fcl
```
**scipy**:
```bash
sudo pip install scipy
```
**Pyyaml**:
```bash
sudo pip install PyYaml
```
**Pypcd**: https://github.com/dimatura/pypcd
Lastly, update your version of **numpy**:
```bash
sudo pip install --upgrade numpy
```

### Rust Dependencies
To use this wrapper, you will first need to install Rust. Please go to https://www.rust-lang.org/learn/get-started for more infomation.

## Getting Started

1. Make sure that you have installed all the dependencies.
1. Clone this repo to the *src* directory in your catkin workspace.
1. Build your catkin workspace by using `catkin_make` in the workspace root directory. 
1. Initialize relaxed_ik_core (the Rust library of Relaxed IK) as a submodule by running the following command from the project directory:
    ```bash
    git submodule update --init
    ``` 
1. Navigate to the *relaxed_ik_core* folder and go through the steps below to get relaxed_ik_core ready.
    1. If your robot is in this list: [baxter, hubo, iiwa7, jaco7, panda, sawyer, ur5, yumi], ignore this step. Else, you will need to clone [this repo](https://github.com/uwgraphics/relaxed_ik) and follow the step-by-step guide [there](https://github.com/uwgraphics/relaxed_ik/blob/dev/src/start_here.py) to get the required robot config files into corresponding folders in the *config* folder in the core. To specify, there should be (replace "sawyer" with your robot name or your urdf name in some cases):
        - 1 self-collision file <collision_sawyer.yaml> in the *collision_files* folder
        - 4 Rust neural network files <sawyer_nn, sawyer_nn.yaml, sawyer_nn_jointpoint, sawyer_nn_jointpoint.yaml> in the *collision_nn_rust* folder
        - 1 info file <sawyer_info.yaml> in the *info_files* folder
        - 1 joint state function file <sawyer_joint_state_define> in the *joint_state_define_functions* folder
        - 1 urdf file <sawyer.urdf> in the *urdfs* folder.
    1. Compile the core:
        ```bash
        cargo build
        ```
1. Look at <settings.yaml> in the folder *relaxed_ik_core/config* and follow the instructions there to customize the parameters, change the input device, and manage the environment obstacles. Note that you don't need to recompile *relaxed_ik_core* every time you change the parameters in <settings.yaml>.
1. View the robot arm in rviz by typing the following command:
    ```bash
    roslaunch relaxed_ik_ros1 rviz_viewer.launch
    ```
1. Launch the Relaxed IK solver by typing the following command:
    ```bash
    roslaunch relaxed_ik_ros1 relaxed_ik_rust.launch
    ```
1. Set a ROS parameter to start the simulation:
    ```bash
    rosparam set /simulation_time go
    ```
1. **[For Testing purposes]** Control the robot based on the type of input device in <settings.yaml>. If you set the robot follow a given cartesian trajectories, you should see the robot moving now! Some examples of cartesian trajectories are provided in the folder *animation_files*. If you set the `input_device` to be keyboard, initialize the keyboard IK goal driver in a new terminal. The driver listens to 6-DOF pose goals and publishes robot joint angles.
    ```bash
    rosrun relaxed_ik_ros1 keyboard_ikgoal_driver.py
    ```
    To use the keyboard controller, please ensure that the termainal window where <keyboard_ikgoal_driver.py> was run from has focus (i.e., make sure it's clicked), then use the following keystrokes:
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

    i - move chain 2 along +X
    m - move chain 2 along -X
    j - move chain 2 along +Y
    l - move chain 2 along -Y
    u - move chain 2 along +Z
    n - move chain 2 along -Z
    = - rotate chain 2 around +X
    - - rotate chain 2 around -X
    0 - rotate chain 2 around +Y
    9 - rotate chain 2 around -Y
    8 - rotate chain 2 around +Z
    7 - rotate chain 2 around -Z
    ```

### Existing Issues
- This wrapper only support the XYZ-like point cloud file format currently. Please refer to the geometry files in the folder *geometry_files* for some examples.