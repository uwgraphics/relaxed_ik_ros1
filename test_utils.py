#! /usr/bin/env python

import numpy
import transformations as T
from geometry_msgs.msg import Pose

def is_point(pt):
    if len(pt) < 3:
        return False
    for e in pt:
        try:
            float(e)
        except ValueError:
            return False
    return True
    
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

def get_abs_waypoints(relative_waypoints, init_pose):
    waypoints = []
    for relative_goal in relative_waypoints:
        relative_orientation_goal = [relative_goal.orientation.w, relative_goal.orientation.x, relative_goal.orientation.y, relative_goal.orientation.z]
        
        goal = Pose()
        goal.position.x = relative_goal.position.x + init_pose.position.x
        goal.position.y = relative_goal.position.y + init_pose.position.y
        goal.position.z = relative_goal.position.z + init_pose.position.z

        init_orientation = [init_pose.orientation.w, init_pose.orientation.x, init_pose.orientation.y, init_pose.orientation.z]
        goal_orientation = T.quaternion_multiply(relative_orientation_goal, init_orientation)
        goal.orientation.w = goal_orientation[0]
        goal.orientation.x = goal_orientation[1]
        goal.orientation.y = goal_orientation[2]
        goal.orientation.z = goal_orientation[3]
        waypoints.append(goal)

    return waypoints

def linear_interpolate_waypoints(waypoints, keyframe):
    if keyframe < 0 or keyframe > len(waypoints) - 1:
        return

    floor = int(keyframe)
    # if (keyframe - floor == 0.0):
    #     return waypoints[floor]
        
    wpose = Pose()
    dx = waypoints[floor + 1].position.x - waypoints[floor].position.x
    wpose.position.x = waypoints[floor].position.x + (keyframe - floor) * dx
    dy = waypoints[floor + 1].position.y - waypoints[floor].position.y
    wpose.position.y = waypoints[floor].position.y + (keyframe - floor) * dy
    dz = waypoints[floor + 1].position.z - waypoints[floor].position.z
    wpose.position.z = waypoints[floor].position.z + (keyframe - floor) * dz

    q0 = [waypoints[floor].orientation.w, waypoints[floor].orientation.x, waypoints[floor].orientation.y, waypoints[floor].orientation.z]
    q1 = [waypoints[floor + 1].orientation.w, waypoints[floor + 1].orientation.x, waypoints[floor + 1].orientation.y, waypoints[floor + 1].orientation.z]
    q = T.unit_vector(T.quaternion_slerp(q0, q1, keyframe - floor))
    wpose.orientation.w = q[0]
    wpose.orientation.x = q[1]
    wpose.orientation.y = q[2]
    wpose.orientation.z = q[3]

    return wpose

def linear_interpolate_joint_states(ja_stream_list, output_size):
    if output_size == len(ja_stream_list): return ja_stream_list
    ja_stream = numpy.array(ja_stream_list)
    new_ja_stream = []
    if output_size > len(ja_stream):
        ja_per_interval = (output_size - len(ja_stream)) / (len(ja_stream) - 1)
        ja_extra = (output_size - len(ja_stream)) % (len(ja_stream) - 1)
        for i, ja in enumerate(ja_stream):
            if i == 0: 
                new_ja_stream.append(list(ja))
                continue
            add_ja_number = ja_per_interval
            if i <= ja_extra:
                add_ja_number += 1
            for j in range(add_ja_number):
                step = (ja - ja_stream[i - 1]) * float(j + 1) / float(add_ja_number + 1)
                new_ja_stream.append(list(ja_stream[i - 1] + step))
            new_ja_stream.append(list(ja))
    else:
        ja_per_interval = (len(ja_stream) - output_size) / (output_size - 1)
        ja_extra = (len(ja_stream) - output_size) % (output_size - 1)
        i = 0
        while i < len(ja_stream):
            print(i)
            new_ja_stream.append(list(ja_stream[i]))
            if i < ja_extra:
                i += ja_per_interval + 2
            else:
                i += ja_per_interval + 1

    print(len(ja_stream), len(new_ja_stream))
    return new_ja_stream

def benchmark_evaluate(ja_stream_list):
    ja_stream = numpy.array(ja_stream_list)
    v_sum = 0.0
    a_sum = 0.0
    jerk_sum = 0.0
    for i, x in enumerate(ja_stream):
        if i == 0: continue
        v1 = x - ja_stream[i-1]
        if i == 1:
            v_sum += v1
            continue
        v2 = ja_stream[i-1] - ja_stream[i-2]
        a1 = v2 - v1
        if i == 2:
            a_sum += a1
            continue
        v3 = ja_stream[i-2] - ja_stream[i-3]
        a2 = v3 - v2
        jerk_sum += a2 - a1
    
    v_aver = v_sum / (len(ja_stream) - 1)
    a_aver = a_sum / (len(ja_stream) - 2)
    jerk_aver = jerk_sum / (len(ja_stream) - 3)
    
    print(v_aver, a_aver, jerk_aver)
