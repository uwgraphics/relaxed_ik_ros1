#!/bin/bash

CONFIG="$(pwd)/relaxed_ik_core/config" 
RMOS="$(pwd)/rmos_files"
ITER=5

declare -a robots=("iiwa7" "ur5" "jco7" "sawyer" "hubo8")

for i in "${robots[@]}"; do
    echo -n "${i}_info.yaml" > "$CONFIG/loaded_robot"
    for f in $RMOS/$i/*; do
        for j in 1 2 3 .. $ITER; do
            gnome-terminal -- /bin/bash -c "roslaunch relaxed_ik_ros1 moveit_test.launch robot:=${i}; exec bash"
            sleep 2
            echo -n $(basename -- "$f") > "$CONFIG/env_collision"
            gnome-terminal -- /bin/bash -c "roslaunch relaxed_ik_ros1 rviz_viewer.launch; exec bash"
            sleep 3
            gnome-terminal --wait -- /bin/bash -c "rosrun relaxed_ik_ros1 moveit_test.py"
        done
    done
done
