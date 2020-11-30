#! /usr/bin/env python

import csv
import numpy
import os
import rospkg
import transformations as T
import yaml

from geometry_msgs.msg import Pose
from robot import Robot
from urdf_load import urdf_load

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')

def is_point(pt):
    if len(pt) < 3:
        return False
    for e in pt:
        try:
            float(e)
        except ValueError:
            return False
    return True

def get_ee_link(info_file_name):
    name = info_file_name.split('_')[0]
    ee_dict = {"ur5": ["ee_link"], "iiwa7": ["iiwa_link_ee"], "jaco7": ["j2s7s300_end_effector"], "hubo8": ["Body_RHAND"]}
    return ee_dict.get(name)
    
def read_cartesian_path(filename, scale=1.0):
    file = open(filename, 'r')
    lines = file.readlines()
    waypoints = []
    for line in lines:
        line = line.strip()
        info = line.split(';')
        time = float(info[0])
        position = info[1].split(',')
        orientation = info[2].split(',')

        wpose = Pose()
        wpose.position.x = float(position[0]) * scale
        wpose.position.y = float(position[1]) * scale
        wpose.position.z = float(position[2]) * scale
        wpose.orientation.w = float(orientation[0])
        wpose.orientation.x = float(orientation[1])
        wpose.orientation.y = float(orientation[2])
        wpose.orientation.z = float(orientation[3])
        waypoints.append((time, wpose))

    return waypoints

def get_abs_waypoints(relative_waypoints, init_pose):
    waypoints = []
    for (time, relative_goal) in relative_waypoints:
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
        waypoints.append((time, goal))

    return waypoints

def linear_interpolate_waypoints(waypoints, keyframe):
    if keyframe >= len(waypoints) - 1:
        return waypoints[-1]

    floor = int(keyframe)
        
    wpose = Pose()
    dx = waypoints[floor + 1][1].position.x - waypoints[floor][1].position.x
    wpose.position.x = waypoints[floor][1].position.x + (keyframe - floor) * dx
    dy = waypoints[floor + 1][1].position.y - waypoints[floor][1].position.y
    wpose.position.y = waypoints[floor][1].position.y + (keyframe - floor) * dy
    dz = waypoints[floor + 1][1].position.z - waypoints[floor][1].position.z
    wpose.position.z = waypoints[floor][1].position.z + (keyframe - floor) * dz

    q0 = [waypoints[floor][1].orientation.w, waypoints[floor][1].orientation.x, waypoints[floor][1].orientation.y, waypoints[floor][1].orientation.z]
    q1 = [waypoints[floor + 1][1].orientation.w, waypoints[floor + 1][1].orientation.x, waypoints[floor + 1][1].orientation.y, waypoints[floor + 1][1].orientation.z]
    q = T.unit_vector(T.quaternion_slerp(q0, q1, keyframe - floor))
    wpose.orientation.w = q[0]
    wpose.orientation.x = q[1]
    wpose.orientation.y = q[2]
    wpose.orientation.z = q[3]

    dt = waypoints[floor + 1][0] - waypoints[floor][0]
    time = waypoints[floor][0] + (keyframe - floor) * dt

    return (time, wpose)

def linear_interpolate_joint_states(ja_stream_list, times):
    if times <= 1: return ja_stream_list
    ja_stream = numpy.array(ja_stream_list)
    new_ja_stream = []
    ja_per_interval = times - 1
    
    for i, ja in enumerate(ja_stream):
        if i == 0: 
            new_ja_stream.append(list(ja))
            continue
        for j in range(ja_per_interval):
            delta = (ja - ja_stream[i - 1]) * float(j + 1) / float(ja_per_interval + 1)
            new_ja_stream.append(list(ja_stream[i - 1] + delta))
        new_ja_stream.append(list(ja))

    # print("The original joint state stream of size {} is interpolated as {} waypoints"\
    #       .format(len(ja_stream), len(new_ja_stream)))
    return new_ja_stream

def extract_joint_states(ja_stream_list, step):
    new_ja_stream = []
    i = 0
    while i < len(ja_stream_list):
        new_ja_stream.append(ja_stream_list[i])
        i += step
    # print("The original joint state stream of size {} is interpolated as {} waypoints"\
    #     .format(len(ja_stream_list), len(new_ja_stream)))
    return new_ja_stream

def get_init_pose(info_file_path):
    info_file = open(info_file_path, 'r')
    y = yaml.load(info_file, Loader=yaml.FullLoader)
    starting_config = y['starting_config']
    fixed_ee_joints = y['ee_fixed_joints']
    full_joint_lists = y['joint_names']
    num_chains = len(full_joint_lists)
    joint_order = y['joint_ordering']

    # Set up Relaxed IK Python robot
    arms = []
    for i in range(num_chains):
        urdf_robot, arm, arm_c, tree = urdf_load('', '', '', full_joint_lists[i], fixed_ee_joints[i])
        arms.append(arm)
    robot = Robot(arms, full_joint_lists, joint_order)
    init_trans = robot.get_ee_positions(starting_config)[0]
    init_rot = robot.get_ee_rotations(starting_config)[0]

    return init_trans, init_rot
