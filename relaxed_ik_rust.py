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

from pykdl_utils.kdl_kinematics import KDLKinematics
from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float64
from timeit import default_timer as timer
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

rospack = rospkg.RosPack()
p = rospack.get_path('relaxed_ik_ros1')
os.chdir(p + "/relaxed_ik_core")

lib = ctypes.cdll.LoadLibrary(p + '/relaxed_ik_core/target/debug/librelaxed_ik_lib.so')
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
        fixed_frame = y['fixed_frame']
        starting_config = y['starting_config']
        ee_links = test_utils.get_ee_link(info_file_name)
        if ee_links == None: ee_links = y['ee_fixed_joints']

        # Set up the kinematics chain
        robot = URDF.from_parameter_server()
        kdl_kin = KDLKinematics(robot, fixed_frame, ee_links[0])
        pose = kdl_kin.forward(starting_config)
        init_trans = [pose[0,3], pose[1,3], pose[2,3]]
        init_rot = T.quaternion_from_matrix(pose)

        # Read the cartesian path
        waypoints = test_utils.read_cartesian_path(rospkg.RosPack().get_path('relaxed_ik_ros1') + "/cartesian_path_files/cartesian_path_prototype")
        
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
        goal_idx = 0
        cur_time = 0.0
        delta_time = 0.01
        stuck_count = 0
        max_time = len(waypoints) * delta_time * 5.0
        ja_stream = []
        prev_sol = starting_config
        
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
            start = timer()
            xopt = lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))
            end = timer()
            print("Speed: {}".format(1.0 / (end - start)))
            # Publish the joint angle solution
            ja = JointAngles()
            for i in range(xopt.length):
                ja.angles.data.append(xopt.data[i])
            angles_pub.publish(ja)
            # Calculate the distance and the angle between the current state and the current goal
            pose = kdl_kin.forward(ja.angles.data)
            trans_cur = [pose[0,3], pose[1,3], pose[2,3]]
            rot_cur = T.quaternion_from_matrix(pose)
            print("Current position: {}\nCurrent orientation: {}".format(list(trans_cur), list(rot_cur)))
            trans_goal = numpy.array(init_trans) + numpy.array([p.position.x, p.position.y, p.position.z])
            rot_goal = T.quaternion_multiply([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z], init_rot)
            print("Current goal position: {}\nCurrent goal orientation: {}".format(list(trans_goal), list(rot_goal)))
            dis = numpy.linalg.norm(numpy.array(trans_cur) - numpy.array(trans_goal))
            angle_between = numpy.linalg.norm(T.quaternion_disp(rot_cur, rot_goal)) * 2.0
            print(dis, angle_between)
            # Advance the clock
            pos_goal_tolerance = 0.01
            quat_goal_tolerance = 0.01
            if dis < pos_goal_tolerance and angle_between < quat_goal_tolerance:
                ja_stream.append(ja.angles.data)
                cur_time += delta_time
                if goal_idx < len(waypoints) - 1:
                    goal_idx += 1
                else:
                    break
            # Check if relaxed Ik gets stuck in local minimum
            v_norm = numpy.linalg.norm(numpy.array(ja.angles.data) - numpy.array(prev_sol))
            prev_sol = ja.angles.data
            # print(v_norm)
            if v_norm < 0.0001:
                stuck_count += 1
            else:
                stuck_count = 0
            
            if stuck_count > 20 and cur_time > len(waypoints) * delta_time:
                break

            rate.sleep()

        print("\nThe path is planned to take {} seconds and in practice it takes {} seconds".format(len(waypoints) * delta_time, cur_time))
        print("Size of the joint state stream: {}".format(len(ja_stream)))
        # test_utils.benchmark_evaluate(ja_stream, 0.1)
        
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
