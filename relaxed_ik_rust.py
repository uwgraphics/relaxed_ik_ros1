#! /usr/bin/env python

import ctypes
import numpy
import os
import rospkg
import rospy
import sys
import test_utils
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

rospack = rospkg.RosPack()
package_path = rospack.get_path('relaxed_ik_ros1')
os.chdir(package_path + "/relaxed_ik_core")

lib = ctypes.cdll.LoadLibrary(package_path + '/relaxed_ik_core/target/debug/librelaxed_ik_lib.so')
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
    # Load the infomation
    args = rospy.myargv(argv=sys.argv)
    print("\nSolver initialized!\nSolver mode: {}".format(args[1]))
    rospy.init_node('relaxed_ik')
    
    # Publishers
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
    time_pub = rospy.Publisher('/relaxed_ik/current_time', Float64, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb)
    rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb)
    
    # If the solver mode is cartesian_path
    if args[1] == "cartesian_path": 
        # Load relevant information
        path_to_src = os.path.dirname(__file__)
        info_file_name = open(path_to_src + '/relaxed_ik_core/config/loaded_robot', 'r').read()
        info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
        info_file = open(info_file_path, 'r')
        y = yaml.load(info_file)
        starting_config = y['starting_config']
        fixed_ee_joints = y['ee_fixed_joints']
        ee_links = test_utils.get_ee_link(info_file_name)
        if ee_links == None: ee_links = fixed_ee_joints
        full_joint_lists = y['joint_names']
        joint_order = y['joint_ordering']
        num_chains = len(full_joint_lists)
        mode = open(path_to_src + '/rmos_files/objective_mode', 'r').read()

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
        env_collision_file_path = path_to_src + '/rmos_files/test.rmos'
        if os.path.exists(env_collision_file_path):
            with open(env_collision_file_path, 'r') as env_collision_file:
                lines = env_collision_file.read().split('\n')
                data_no_comment = lines[0].split('//')
                data = data_no_comment[0].strip().split(';')
                cartesian_path_file_name = data[-1]
        waypoints = test_utils.read_cartesian_path(rospkg.RosPack()\
            .get_path('relaxed_ik_ros1') + "/cartesian_path_files/" + cartesian_path_file_name, scale=1.0)
        final_trans_goal = numpy.array(init_trans) + numpy.array([waypoints[-1][1].position.x, \
            waypoints[-1][1].position.y, waypoints[-1][1].position.z])
        final_rot_goal = T.quaternion_multiply([waypoints[-1][1].orientation.w, waypoints[-1][1].orientation.x, \
            waypoints[-1][1].orientation.y, waypoints[-1][1].orientation.z], init_rot)
        
        # Wait for the start signal
        print("Waiting for ROS param /exp_status to be set as go...")
        initialized = False
        while not initialized: 
            try: 
                param = rospy.get_param("exp_status")
                initialized = param == "go"
            except KeyError:
                initialized = False

        # Set up the initial parameters
        step = 1 / 30.0
        goal_idx = 0
        cur_time = 0.0
        delta_time = 0.01
        stuck_count = 0
        max_time = len(waypoints) * delta_time * 5.0
        ja_stream = []
        prev_sol = starting_config
        num_collisions = 0
        
        rate = rospy.Rate(3000)
        while not rospy.is_shutdown():
            if cur_time >= max_time: break
            # Publish the current time
            cur_time_msg = Float64()
            cur_time_msg.data = cur_time
            time_pub.publish(cur_time_msg)
            
            # Get the pose goal
            (time, p) = test_utils.linear_interpolate_waypoints(waypoints, goal_idx)
            pos_arr = (ctypes.c_double * 3)()
            quat_arr = (ctypes.c_double * 4)()
            pos_arr[0] = p.position.x
            pos_arr[1] = p.position.y
            pos_arr[2] = p.position.z
            quat_arr[0] = p.orientation.x
            quat_arr[1] = p.orientation.y
            quat_arr[2] = p.orientation.z
            quat_arr[3] = p.orientation.w
            
            # Solve
            # start = timer()
            xopt = lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))
            # end = timer()
            # print("Speed: {}".format(1.0 / (end - start)))
            
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
            print(dis, angle_between)
            
            # # Advance the clock
            pos_goal_tolerance = 0.01
            quat_goal_tolerance = 0.01
            if dis < pos_goal_tolerance and (angle_between < quat_goal_tolerance or mode == 'ECA3'):
                print("The path is finished successfully!")
                break

            # Check if relaxed Ik gets stuck in local minimum
            v_norm = numpy.linalg.norm(numpy.array(ja.angles.data) - numpy.array(prev_sol))
            prev_sol = ja.angles.data
            # print(v_norm)
            if v_norm == 0.0:
                num_collisions += 1
                stuck_count += 1
            elif v_norm < 0.001:
                stuck_count += 1
            else:
                stuck_count = 0
            
            if stuck_count > 20 / step and cur_time > len(waypoints) * delta_time:
                print("relaxed IK is stucked in local minimum!")
                break

            rate.sleep()

        robot_name = info_file_name.split('_')[0]
        benchmark_evaluator = test_utils.BenchmarkEvaluator(waypoints, ja_stream, delta_time, step, \
            package_path + "/rmoo_files", "relaxed_ik" + '_' + mode, robot_name)
        benchmark_evaluator.write_ja_stream(interpolate=True)
        v_avg, a_avg, jerk_avg = benchmark_evaluator.calculate_joint_stats(interpolate=False)
        pos_error_avg, rot_error_avg = benchmark_evaluator.calculate_error_stats(interpolate=False)

        robot_str = "Robot: {}\n".format(robot_name)
        software_str = "Software: Relaxed IK\n"
        mode_str = "Mode: {}\n".format(mode)
        motion_time_str = "Planned motion length: {}\nActual motion length: {}\n".format(len(waypoints) * delta_time, cur_time)
        joint_stream_str = "Size of the joint state stream: {}/{}\n".format(int((len(ja_stream) - 1) * step) + 1, len(ja_stream))
        joint_stats_str = "Average joint velocity: {}\nAverage joint acceleration: {}\nAverage joint jerk: {}\n"\
            .format(v_avg, a_avg, jerk_avg)
        err_stats_str = "Average position error: {}\nAverage rotation error: {}\n".format(pos_error_avg, rot_error_avg)
        num_collisions_str = "Number of environment collisions: {}".format(num_collisions)
        test_result = robot_str + software_str + mode_str + motion_time_str + joint_stream_str + joint_stats_str + err_stats_str + num_collisions_str
        print(test_result)

        rmob_file_path = package_path + '/rmob_files/' + robot_name + '/' + robot_name + '_relaxed_ik_' + mode + '.rmob'
        with open(rmob_file_path, 'w') as rmob_file:
            rmob_file.write(test_result)

    # When the input is keyboard
    elif args[1] == "keyboard":
        global eepg
        rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)

        while eepg == None: continue

        rate = rospy.Rate(3000)
        while not rospy.is_shutdown():
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
            print("Speed: {}".format(1.0 / (end - start)))

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

if __name__ == '__main__':
    main()
