#! /usr/bin/env python

import os
import rospkg

rospack = rospkg.RosPack()
p = rospack.get_path('relaxed_ik_ros1')
os.chdir(p + "/relaxed_ik_core")
os.system('cargo run --bin relaxed_ik_node_ros1')
