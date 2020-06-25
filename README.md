# relaxed_ik_ros1

This is Relaxed IK wrapped up in ROS1.

## Run
1. Configure the name of the pre-computed robot you would like to run with  (available options are ur5, yumi, panda and iiwa7) in relaxed_ik_core/config/loaded_robot.

2. Run the following command:
```
roslaunch relaxed_ik_ros1 relaxed_ik_rust.launch
```

3. Test by opening a new terminal and publishing a new message to the topic EE pose goals:
```
rostopic pub -1 /relaxed_ik/ee_pose_goals relaxed_ik/EEPoseGoals '[0, now, base_link]' '[{position: [0.015, 0.015, 0.015], orientation: [0.0, 0.0, 0.0, 1.0]}]'
```

## Known issues
1. This wrapper is pointing to a different commit from the ROS2 and Unity wrappers because of a different Cargo.toml.