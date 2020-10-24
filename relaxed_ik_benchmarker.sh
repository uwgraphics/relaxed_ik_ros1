#!/bin/bash

# clear previous outputs if any
./test_output_clear.sh

CONFIG="$(pwd)/relaxed_ik_core/config" 
RMOS="$(pwd)/rmos_files"

declare -a robots=("hubo8" "jaco7" "iiwa7" "sawyer" "ur5")
declare -a modes=("ECA" "noECA" "ECA3" "ECAA")

for i in "${robots[@]}"; do
    echo -n "${i}_info.yaml" > "$CONFIG/loaded_robot"
    for f in $RMOS/$i/*; do
        echo -n $(basename -- "$f") > "$CONFIG/env_collision"
        gnome-terminal -- /bin/bash -c "roslaunch relaxed_ik_ros1 rviz_viewer.launch; exec bash"
        sleep 1
        for m in "${modes[@]}"; do
            rosparam set /exp_status go
            echo -n "$m" > "$CONFIG/objective_mode"
            gnome-terminal --wait -- /bin/bash -c "roslaunch --wait relaxed_ik_ros1 relaxed_ik_rust.launch"
        done
    done
done
