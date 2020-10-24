#! /usr/bin/env python

import csv
import numpy
import os
import transformations as T
import yaml

from geometry_msgs.msg import Pose
from robot import Robot
from urdf_load import urdf_load

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

def get_group_name(info_file_name):
    name = info_file_name.split('_')[0]
    ee_dict = {"ur5": "manipulator", "sawyer": "right_arm", "iiwa7": "manipulator", "jaco7": "manipulator", "hubo8": "right_arm"}
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
    # if (keyframe - floor == 0.0):
    #     return waypoints[floor]
        
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
    #     .format(len(ja_stream), len(new_ja_stream)))
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

class BenchmarkEvaluator:
    def __init__(self, waypoints, ja_stream, delta_time, step, root, interface, robot, test_name):
        self.waypoints = waypoints
        self.ja_stream = numpy.array(ja_stream)
        self.delta_time = delta_time
        self.step = step
        self.root = root
        self.interface = interface
        self.robot = robot
        self.test_name = test_name

    def write_ja_stream(self, interpolate=True):
        new_ja_stream = self.ja_stream
        if interpolate:
            new_ja_stream = extract_joint_states(self.ja_stream, int(1 / self.step))
        path = self.root + '/' + self.robot + '/' + self.test_name + '_' + self.interface + '.rmoo'
        with open(path, "w") as file:
            for i, ja in enumerate(new_ja_stream):
                file.write('{};{}\n'.format(i * self.delta_time, ','.join(str(n) for n in ja)))
    
    def calculate_joint_stats(self, interpolate=False):
        new_ja_stream = self.ja_stream
        interval = self.delta_time * self.step
        if interpolate:
            new_ja_stream = extract_joint_states(self.ja_stream, int(1 / self.step))
            interval = self.delta_time
        
        v_sum = 0.0
        a_sum = 0.0
        jerk_sum = 0.0
        joint_velocities = []
        joint_accelerations = []
        joint_jerks = []    
        for i, x in enumerate(new_ja_stream):
            if i == 0: continue
            v1 = x - new_ja_stream[i-1]
            joint_velocities.append(v1)
            # v_sum += numpy.linalg.norm(v1)
            if i == 1: continue
            v2 = new_ja_stream[i-1] - new_ja_stream[i-2]
            a1 = v1 - v2
            joint_accelerations.append(a1)
            # a_sum += numpy.linalg.norm(a1)
            if i == 2: continue
            v3 = new_ja_stream[i-2] - new_ja_stream[i-3]
            a2 = v2 - v3
            j1 = a1 - a2
            joint_jerks.append(j1)
            # jerk_sum += numpy.linalg.norm(j1)

        for v in joint_velocities:
            v /= interval
            v_sum += numpy.linalg.norm(v)

        for a in joint_accelerations:
            a /= interval**2
            a_sum += numpy.linalg.norm(a)
        
        for j in joint_jerks:
            j /= interval**3
            jerk_sum +=  numpy.linalg.norm(j)
        
        v_avg = v_sum / (len(new_ja_stream) - 1)
        a_avg = a_sum / (len(new_ja_stream) - 2)
        jerk_avg = jerk_sum / (len(new_ja_stream) - 3)

        # file_names = [('velocities', joint_velocities), ('accelerations', joint_accelerations), \
        #     ('jerks', joint_jerks)]
        # for name, array in file_names:
        #     path = self.root + '/' + self.robot + '/joint_' + name + '_' + self.interface
        #     with open(path, "w") as file:
        #         writer = csv.writer(file, delimiter=',')
        #         writer.writerows(array)

        return v_avg, a_avg, jerk_avg
        
    def calculate_error_stats(self, interpolate=False):
        path_to_src = os.path.dirname(__file__)
        info_file_name = open(path_to_src + '/relaxed_ik_core/config/loaded_robot', 'r').read()
        info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
        info_file = open(info_file_path, 'r')
        y = yaml.load(info_file)
        starting_config = y['starting_config']
        fixed_ee_joints = y['ee_fixed_joints']
        full_joint_lists = y['joint_names']
        joint_order = y['joint_ordering']
        num_chains = len(full_joint_lists)

        # Set up Relaxed IK Python robot
        arms = []
        for i in range(num_chains):
            urdf_robot, arm, arm_c, tree = urdf_load('', '', '', full_joint_lists[i], fixed_ee_joints[i])
            arms.append(arm)
        robot = Robot(arms, full_joint_lists, joint_order)
        init_trans = robot.get_ee_positions(starting_config)[0]
        init_rot = robot.get_ee_rotations(starting_config)[0]
        
        new_ja_stream = self.ja_stream
        step = self.step
        if interpolate:
            new_ja_stream = extract_joint_states(self.ja_stream, int(1 / self.step))
            step = 1
        pos_error_sum = 0.0
        rot_error_sum = 0.0
        goal_idx = 0
        for ja in new_ja_stream:
            trans_cur = robot.get_ee_positions(ja)[0]
            rot_cur = robot.get_ee_rotations(ja)[0]

            (time, p) = linear_interpolate_waypoints(self.waypoints, goal_idx)
            trans_goal = numpy.array(init_trans) + numpy.array([p.position.x, p.position.y, p.position.z])
            rot_goal = T.quaternion_multiply([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z], init_rot)

            pos_error_sum += numpy.linalg.norm(numpy.array(trans_cur) - numpy.array(trans_goal))
            rot_error_sum += numpy.linalg.norm(T.quaternion_disp(rot_cur, rot_goal)) * 2.0
            
            goal_idx += step

        pos_error_avg = pos_error_sum / len(new_ja_stream)
        rot_error_avg = rot_error_sum / len(new_ja_stream)

        return pos_error_avg, rot_error_avg
