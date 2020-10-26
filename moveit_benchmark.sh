#!/bin/bash

CONFIG="$(pwd)/relaxed_ik_core/config" 
RMOS="$(pwd)/rmos_files"

declare -a robots=("ur5" "jaco7" "sawyer" "iiwa7" "hubo8")

for i in "${robots[@]}"; do
    echo -n "${i}_info.yaml" > "$CONFIG/loaded_robot"
    for f in $RMOS/$i/*; do
        for j in {1..2}; do
            gnome-terminal -- /bin/bash -c "roslaunch relaxed_ik_ros1 moveit_test.launch robot:=${i}; exec bash"
            sleep 2
            echo -n $(basename -- "$f") > "$CONFIG/env_collision"
            gnome-terminal -- /bin/bash -c "roslaunch relaxed_ik_ros1 rviz_viewer.launch; exec bash"
            rosparam set /exp_status go
            sleep 3
            gnome-terminal --wait -- /bin/bash -c "rosrun relaxed_ik_ros1 moveit_test.py ${j}"
        done
    done
done
