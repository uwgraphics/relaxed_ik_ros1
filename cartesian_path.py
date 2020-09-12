#! /usr/bin/env python

import math
import transformations as T
from geometry_msgs.msg import Pose

def read_cartesian_path(filename):
    file = open(filename, 'r')
    lines = file.readlines()
    waypoints = []
    for line in lines:
        line = line.strip()
        info = line.split(';')
        position = info[1].split(',')
        orientation = info[2].split(',')

        wpose = Pose()
        wpose.position.x = float(position[0])
        wpose.position.y = float(position[1])
        wpose.position.z = float(position[2])
        wpose.orientation.w = float(orientation[0])
        wpose.orientation.x = float(orientation[1])
        wpose.orientation.y = float(orientation[2])
        wpose.orientation.z = float(orientation[3])
        waypoints.append(wpose)

    return waypoints

def linear_interpolate(waypoints, keyframe):
    if keyframe < 0 or keyframe > len(waypoints) - 1:
        return
    wpose = Pose()
    floor = int(keyframe)
    dx = waypoints[floor + 1].position.x - waypoints[floor].position.x
    wpose.position.x = waypoints[floor].position.x + (keyframe - floor) * dx
    dy = waypoints[floor + 1].position.y - waypoints[floor].position.y
    wpose.position.y = waypoints[floor].position.y + (keyframe - floor) * dy
    dz = waypoints[floor + 1].position.z - waypoints[floor].position.z
    wpose.position.z = waypoints[floor].position.z + (keyframe - floor) * dz

    q0 = [waypoints[floor].orientation.w, waypoints[floor].orientation.x, waypoints[floor].orientation.y, waypoints[floor].orientation.z]
    q1 = [waypoints[floor + 1].orientation.w, waypoints[floor + 1].orientation.x, waypoints[floor + 1].orientation.y, waypoints[floor + 1].orientation.z]
    q = T.quaternion_slerp(q0, q1, keyframe - floor)
    wpose.orientation.w = q[0]
    wpose.orientation.x = q[1]
    wpose.orientation.y = q[2]
    wpose.orientation.z = q[3]

    return wpose
