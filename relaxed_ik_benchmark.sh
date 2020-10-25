#!/bin/bash

CONFIG="$(pwd)/relaxed_ik_core/config" 
RMOS="$(pwd)/rmos_files"

declare -a robots=("ur5" "jaco7" "iiwa7" "sawyer" "hubo8")
declare -a modes=("ECA" "noECA" "ECA3" "ECAA")

for i in "${robots[@]}"; do
    echo -n "${i}_info.yaml" > "$CONFIG/loaded_robot"
    for f in $RMOS/$i/*; do
        echo -n $(basename -- "$f") > "$CONFIG/env_collision"
        gnome-terminal -- /bin/bash -c "roslaunch relaxed_ik_ros1 rviz_viewer.launch; exec bash"
        sleep 2
        for m in "${modes[@]}"; do
            rosparam set /exp_status go
            echo -n "$m" > "$CONFIG/objective_mode"
            gnome-terminal --wait -- /bin/bash -c "roslaunch --wait relaxed_ik_ros1 relaxed_ik_rust.launch"
        done
    done
done
