# relaxed_ik_ros1

This is Relaxed IK wrapped up in ROS1.

## Run
1. Configure the name of the pre-computed robot you would like to run with  (available options are ur5, yumi, panda and iiwa7) in relaxed_ik_core/config/loaded_robot.
2. Run the following command:
```
roslaunch relaxed_ik_ros1 relaxed_ik_rust.launch
```