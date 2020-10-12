#! /usr/bin/env python

import csv
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
            new_ja_stream.append(list(ja_stream[i]))
            if i < ja_extra:
                i += ja_per_interval + 2
            else:
                i += ja_per_interval + 1

    print("The original joint state stream of size {} is interpolated as {}/{} waypoints"\
        .format(len(ja_stream), len(new_ja_stream), output_size))
    return new_ja_stream

def extract_joint_states(ja_stream_list, step):
    new_ja_stream = []
    i = 0
    while i < len(ja_stream_list):
        new_ja_stream.append(ja_stream_list[i])
        i += step
    print("The original joint state stream of size {} is interpolated as {} waypoints"\
        .format(len(ja_stream_list), len(new_ja_stream)))
    return new_ja_stream

class BenchmarkEvaluator:
    def __init__(self, ja_stream, interval, root, interface, robot):
        self.ja_stream = numpy.array(ja_stream)
        self.interval = interval
        self.root = root
        self.interface = interface
        self.robot = robot

    def write_ja_stream(self):
        path = self.root + '/' + self.robot + '/joint_states_' + self.interface
        with open(path, "w") as file:
            writer = csv.writer(file, delimiter=',')
            writer.writerows(self.ja_stream)
    
    def calculate_joint_stats(self):
        v_sum = 0.0
        a_sum = 0.0
        jerk_sum = 0.0
        joint_velocities = []
        joint_accelerations = []
        joint_jerks = []
        for i, x in enumerate(self.ja_stream):
            if i == 0: continue
            v1 = x - self.ja_stream[i-1]
            joint_velocities.append(v1)
            # v_sum += numpy.linalg.norm(v1)
            if i == 1: continue
            v2 = self.ja_stream[i-1] - self.ja_stream[i-2]
            a1 = v1 - v2
            joint_accelerations.append(a1)
            # a_sum += numpy.linalg.norm(a1)
            if i == 2: continue
            v3 = self.ja_stream[i-2] - self.ja_stream[i-3]
            a2 = v2 - v3
            j1 = a1 - a2
            joint_jerks.append(j1)
            # jerk_sum += numpy.linalg.norm(j1)

        for v in joint_velocities:
            v /= self.interval
            v_sum += numpy.linalg.norm(v)

        for a in joint_accelerations:
            a /= self.interval
            a_sum += numpy.linalg.norm(a)
        
        for j in joint_jerks:
            j /= self.interval
            jerk_sum +=  numpy.linalg.norm(j)
        
        v_aver = v_sum / (len(self.ja_stream) - 1)
        a_aver = a_sum / (len(self.ja_stream) - 2)
        jerk_aver = jerk_sum / (len(self.ja_stream) - 3)
        
        print("Average joint velocity22: {}\nAverage joint acceleration: {}\nAverage joint jerk: {}"\
            .format(v_aver, a_aver, jerk_aver))

        file_names = [('velocities', joint_velocities), ('accelerations', joint_accelerations), \
            ('jerks', joint_jerks)]
        for name, array in file_names:
            path = self.root + '/' + self.robot + '/joint_' + name + '_' + self.interface
            with open(path, "w") as file:
                writer = csv.writer(file, delimiter=',')
                writer.writerows(array)
        
    def calculate_error_stats(self):
        print("Average position errors: {}\nAverage rotation error: {}")