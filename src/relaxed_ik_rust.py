#! /usr/bin/env python

import csv
import ctypes
import numpy
import os
import rospkg
import rospy
import sys
import utils
import transformations as T
import yaml

from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from robot import Robot
from std_msgs.msg import Float64
from timeit import default_timer as timer
from urdf_load import urdf_load
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')
animation_folder_path = path_to_src + '/animation_files/'
env_settings_file_path = path_to_src + '/relaxed_ik_core/config/settings.yaml'

os.chdir(path_to_src + "/relaxed_ik_core")

lib = ctypes.cdll.LoadLibrary(path_to_src + '/relaxed_ik_core/target/debug/librelaxed_ik_lib.so')
lib.solve.restype = Opt

def marker_feedback_cb(msg):
    pos_arr = (ctypes.c_double * 3)()
    quat_arr = (ctypes.c_double * 4)()
    pos_arr[0] = msg.pose.position.x
    pos_arr[1] = msg.pose.position.y
    pos_arr[2] = msg.pose.position.z
    quat_arr[0] = msg.pose.orientation.x
    quat_arr[1] = msg.pose.orientation.y
    quat_arr[2] = msg.pose.orientation.z
    quat_arr[3] = msg.pose.orientation.w
    # Call the rust callback function
    lib.dynamic_obstacle_cb(msg.marker_name, pos_arr, quat_arr)

def marker_update_cb(msg):
    # update dynamic collision obstacles in relaxed IK
    for pose_stamped in msg.poses:
        pos_arr = (ctypes.c_double * 3)()
        quat_arr = (ctypes.c_double * 4)()
        pos_arr[0] = pose_stamped.pose.position.x
        pos_arr[1] = pose_stamped.pose.position.y
        pos_arr[2] = pose_stamped.pose.position.z
        quat_arr[0] = pose_stamped.pose.orientation.x
        quat_arr[1] = pose_stamped.pose.orientation.y
        quat_arr[2] = pose_stamped.pose.orientation.z
        quat_arr[3] = pose_stamped.pose.orientation.w
        # Call the rust callback function
        lib.dynamic_obstacle_cb(pose_stamped.name, pos_arr, quat_arr)

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    rospy.init_node('relaxed_ik')

    # Load the infomation
    env_settings_file = open(env_settings_file_path, 'r')
    env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)

    if 'loaded_robot' in env_settings:
        robot_info = env_settings['loaded_robot']
    else:
        raise NameError('Please define the relevant information of the robot!')

    info_file_name = robot_info['name']
    robot_name = info_file_name.split('_')[0]
    objective_mode = robot_info['objective_mode']
    print("\nRelaxedIK initialized!\nRobot: {}\nObjective mode: {}\n".format(robot_name, objective_mode))
    
    # Publishers
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
    time_pub = rospy.Publisher('/relaxed_ik/current_time', Float64, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb)
    rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb)

    cur_time = 0.0
    delta_time = 0.01
    step = 1 / 30.0

    # Wait for the start signal
    print("Waiting for ROS param /simulation_time to be set as go...")
    initialized = False
    while not initialized: 
        try: 
            param = rospy.get_param("simulation_time")
            initialized = param == "go"
        except KeyError:
            initialized = False
    print("ROS param /simulation_time is set up!\n")

    # If the input_device is keyboard
    if robot_info['input_device'] == 'keyboard': 
        global eepg
        rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)

        print("Waiting for the keyboard_ikgoal_driver being initialized...")
        while eepg == None: continue
        print("The keyboard_ikgoal_driver is initialized!\n")

        rate = rospy.Rate(3000)
        speed_list = []
        while not rospy.is_shutdown():
            cur_time_msg = Float64()
            cur_time_msg.data = cur_time
            time_pub.publish(cur_time_msg)
            cur_time += delta_time * step

            pose_goals = eepg.ee_poses
            header = eepg.header
            pos_arr = (ctypes.c_double * (3 * len(pose_goals)))()
            quat_arr = (ctypes.c_double * (4 * len(pose_goals)))()

            for i in range(len(pose_goals)):
                p = pose_goals[i]
                pos_arr[3*i] = p.position.x
                pos_arr[3*i+1] = p.position.y
                pos_arr[3*i+2] = p.position.z

                quat_arr[4*i] = p.orientation.x
                quat_arr[4*i+1] = p.orientation.y
                quat_arr[4*i+2] = p.orientation.z
                quat_arr[4*i+3] = p.orientation.w

            start = timer()
            xopt = lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))
            end = timer()
            speed = 1.0 / (end - start)
            # print("Speed: {}".format(speed))
            speed_list.append(speed)

            ja = JointAngles()
            ja.header = header
            ja_str = "["
            for i in range(xopt.length):
                ja.angles.data.append(xopt.data[i])
                ja_str += str(xopt.data[i])
                if i == xopt.length - 1:
                    ja_str += "]"
                else: 
                    ja_str += ", "

            angles_pub.publish(ja)
            # print(ja_str)

            rate.sleep()

        print("Average speed: {} HZ".format(numpy.mean(speed_list)))
        print("Min speed: {} HZ".format(numpy.min(speed_list)))
        print("Max speed: {} HZ".format(numpy.max(speed_list)))

    # When the input is animation file
    else:
         # Load relevant information
        info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
        info_file = open(info_file_path, 'r')
        y = yaml.load(info_file, Loader=yaml.FullLoader)
        starting_config = y['starting_config']
        fixed_ee_joints = y['ee_fixed_joints']
        ee_links = utils.get_ee_link(info_file_name)
        if ee_links == None: ee_links = fixed_ee_joints
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
        # print(init_trans, init_rot)

        # Read the cartesian path
        cartesian_path_file_name = robot_info['input_device']
        waypoints = utils.read_cartesian_path(path_to_src + "/animation_files/" + cartesian_path_file_name, scale=1.0)
        final_trans_goal = numpy.array(init_trans) + numpy.array([waypoints[-1][1].position.x, \
            waypoints[-1][1].position.y, waypoints[-1][1].position.z])
        final_rot_goal = T.quaternion_multiply([waypoints[-1][1].orientation.w, waypoints[-1][1].orientation.x, \
            waypoints[-1][1].orientation.y, waypoints[-1][1].orientation.z], init_rot)

        # Set up the initial parameters
        goal_idx = 0
        stuck_count = 0
        max_time = len(waypoints) * delta_time * 2.0
        ja_stream = []
        prev_sol = starting_config
        pos_goal_tolerance = 0.01
        quat_goal_tolerance = 0.01
        
        rate = rospy.Rate(3000)
        while not rospy.is_shutdown():
            if cur_time >= max_time: break
            # Publish the current time
            cur_time_msg = Float64()
            cur_time_msg.data = cur_time
            time_pub.publish(cur_time_msg)
            
            # Get the pose goal
            (time, p) = utils.linear_interpolate_waypoints(waypoints, goal_idx)
            pos_arr = (ctypes.c_double * 3)()
            quat_arr = (ctypes.c_double * 4)()
            pos_arr[0] = p.position.x
            pos_arr[1] = p.position.y
            pos_arr[2] = p.position.z
            quat_arr[0] = p.orientation.x
            quat_arr[1] = p.orientation.y
            quat_arr[2] = p.orientation.z
            quat_arr[3] = p.orientation.w
            
            xopt = lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))
            
            # Publish the joint angle solution
            ja = JointAngles()
            for i in range(xopt.length):
                ja.angles.data.append(xopt.data[i])
            angles_pub.publish(ja)

            ja_stream.append(ja.angles.data)
            cur_time += delta_time * step
            if goal_idx < len(waypoints) - 1:
                goal_idx += 1 * step

            # Calculate the distance and the angle between the current state and the goal
            trans_cur = robot.get_ee_positions(ja.angles.data)[0]
            rot_cur = robot.get_ee_rotations(ja.angles.data)[0]
            # print("Current position: {}\nCurrent orientation: {}".format(list(trans_cur), list(rot_cur)))
            dis = numpy.linalg.norm(numpy.array(trans_cur) - numpy.array(final_trans_goal))
            angle_between = numpy.linalg.norm(T.quaternion_disp(rot_cur, final_rot_goal)) * 2.0
            # print(dis, angle_between)
            
            if dis < pos_goal_tolerance and (angle_between < quat_goal_tolerance or objective_mode == 'ECA3') \
                and cur_time > len(waypoints) * delta_time:
                print("The path is finished successfully!")
                break

            # Check if relaxed Ik gets stuck in local minimum
            v_norm = numpy.linalg.norm(numpy.array(ja.angles.data) - numpy.array(prev_sol))
            prev_sol = ja.angles.data
            # print(v_norm)
            if v_norm < 0.001:
                stuck_count += 1
            else:
                stuck_count = 0
            
            if stuck_count > 20 / step and cur_time > len(waypoints) * delta_time:
                print("relaxed IK is stucked in local minimum!")
                break

            rate.sleep()
        
if __name__ == '__main__':
    main()
