#! /usr/bin/env python

import csv
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
        mode = open(path_to_src + '/relaxed_ik_core/config/objective_mode', 'r').read()
        robot_name = info_file_name.split('_')[0]
        test_name_ext = open(path_to_src + '/relaxed_ik_core/config/env_collision', 'r').read()
        test_name = test_name_ext.split('.')[0]
        print("Robot: {}".format(robot_name))
        print("Objective mode: {}".format(mode))

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
        env_collision_file_name = open(path_to_src + '/relaxed_ik_core/config/env_collision', 'r').read()
        env_collision_file_path = path_to_src + '/rmos_files/' + info_file_name.split('_')[0] + \
            '/' + env_collision_file_name
        if os.path.exists(env_collision_file_path):
            with open(env_collision_file_path, 'r') as env_collision_file:
                lines = env_collision_file.read().split('\n')
                data_no_comment = lines[0].split('//')
                data = data_no_comment[0].strip().split(';')
                cartesian_path_file_name = data[-1]
        waypoints = test_utils.read_cartesian_path(rospkg.RosPack().get_path('relaxed_ik_ros1') + \
            "/animation_files/" + robot_name + "/" + cartesian_path_file_name, scale=1.0)
        final_trans_goal = numpy.array(init_trans) + numpy.array([waypoints[-1][1].position.x, \
            waypoints[-1][1].position.y, waypoints[-1][1].position.z])
        final_rot_goal = T.quaternion_multiply([waypoints[-1][1].orientation.w, waypoints[-1][1].orientation.x, \
            waypoints[-1][1].orientation.y, waypoints[-1][1].orientation.z], init_rot)

        num_collisions_file = open(package_path + "/relaxed_ik_core/num_env_collision", 'w')
        num_collisions_file.seek(0)
        num_collisions_file.write('0')
        num_collisions_file.close()
        
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
            # print(dis, angle_between)
            
            if dis < pos_goal_tolerance and (angle_between < quat_goal_tolerance or mode == 'ECA3') \
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

        benchmark_evaluator = test_utils.BenchmarkEvaluator(waypoints, ja_stream, delta_time, step, \
            package_path + "/rmoo_files", "relaxed_ik" + '_' + mode, robot_name, test_name)
        benchmark_evaluator.write_ja_stream(interpolate=True)
        v_avg, a_avg, jerk_avg = benchmark_evaluator.calculate_joint_stats(interpolate=True)
        pos_error_avg, rot_error_avg = benchmark_evaluator.calculate_error_stats(interpolate=True)

        robot_str = "Robot: {}\n".format(robot_name)
        software_str = "Software: Relaxed IK\n"
        mode_str = "Mode: {}\n".format(mode)
        motion_time = len(waypoints) * delta_time
        motion_time_str = "Planned motion length: {}\nActual motion length: {}\n".format(motion_time, cur_time)
        output_size = int((len(ja_stream) - 1) * step) + 1
        joint_stream_str = "Size of the joint state stream: {}/{}\n".format(output_size, len(ja_stream))
        joint_stats_str = "Average joint velocity: {}\nAverage joint acceleration: {}\nAverage joint jerk: {}\n"\
            .format(v_avg, a_avg, jerk_avg)
        err_stats_str = "Average position error: {}\nAverage rotation error: {}\n".format(pos_error_avg, rot_error_avg)
        num_collisions_file = open(package_path + "/relaxed_ik_core/num_env_collision", 'r')
        num_collisions = num_collisions_file.read()
        num_collisions_file.close()
        num_collisions = int(num_collisions * step)
        num_collisions_str = "Number of environment collisions: {}".format(num_collisions)
        test_result = robot_str + software_str + mode_str + motion_time_str + joint_stream_str + joint_stats_str + err_stats_str + num_collisions_str
        print(test_result)

        rmob_file_path = package_path + '/rmob_files/' + robot_name + '/' + test_name + '_relaxed_ik_' + mode + '.rmob'
        with open(rmob_file_path, 'w') as rmob_file:
            rmob_file.write(test_result)
        
        csv_file_path = package_path + '/csv_files/' + 'master.csv'
        with open(csv_file_path, 'a+') as csv_file:
            field_names = ["robot", "test", "software", "mode", "planned_motion_length", "actual_motion_length", \
                "output_size", "average_joint_velocity", "average_joint_acceleration", \
                "average_joint_jerk", "average_position_error", "average_rotation_error", \
                "num_env_collisions"]
            writer = csv.writer(csv_file, delimiter=',')
            if csv_file.read() == '':
                writer.writerow(field_names)
            writer.writerow([robot_name, test_name, "Relaxed IK", mode, str(motion_time), str(cur_time), str(output_size), \
                str(v_avg), str(a_avg), str(jerk_avg), str(pos_error_avg), str(rot_error_avg), str(num_collisions)])

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
