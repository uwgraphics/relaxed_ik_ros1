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
    # update dynamic collision obstacles in relaxed IK
    pos_arr = (ctypes.c_double * 3)()
    quat_arr = (ctypes.c_double * 4)()

    pos_arr[0] = msg.pose.position.x
    pos_arr[1] = msg.pose.position.y
    pos_arr[2] = msg.pose.position.z

    quat_arr[0] = msg.pose.orientation.x
    quat_arr[1] = msg.pose.orientation.y
    quat_arr[2] = msg.pose.orientation.z
    quat_arr[3] = msg.pose.orientation.w

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
        lib.dynamic_obstacle_cb(pose_stamped.name, pos_arr, quat_arr)

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    args = rospy.myargv(argv=sys.argv)

    print("\nSolver initialized!\nSolver mode: {}".format(args[1]))

    rospy.init_node('relaxed_ik')

    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb)
    rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb)
    
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=3)
    
    if args[1] == "cartesian_path": 
        robot = URDF.from_parameter_server()
        kdl_kin = KDLKinematics(robot, "/base", "/right_hand")

        path_to_src = os.path.dirname(__file__)
        info_file_name = open(path_to_src + '/relaxed_ik_core/config/loaded_robot', 'r').read()
        info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
        info_file = open(info_file_path, 'r')
        y = yaml.load(info_file)
        starting_config = y['starting_config']

        pose = kdl_kin.forward(starting_config)
        init_trans = [pose[0,3], pose[1,3], pose[2,3]]
        init_rot = T.quaternion_from_matrix(pose)
        waypoints = test_utils.read_cartesian_path(rospkg.RosPack().get_path('relaxed_ik_ros1') + "/cartesian_path_files/cartesian_path_prototype")

        initialized = False
        print("Waiting for ROS param /exp_status to set as go...")
        while not initialized: 
            try: 
                param = rospy.get_param("exp_status")
                initialized = param == "go"
            except KeyError:
                initialized = False

        ja_stream = []
        keyframe = 0.0
        step = 0.1
        pos_goal_tolerance = 0.01
        quat_goal_tolerance = 0.01
        trans_cur = init_trans
        rot_cur = init_rot
        rate = rospy.Rate(300)
        while not rospy.is_shutdown():
            p = test_utils.linear_interpolate(waypoints, keyframe)
            pos_arr = (ctypes.c_double * 3)()
            quat_arr = (ctypes.c_double * 4)()

            pos_arr[0] = p.position.x
            pos_arr[1] = p.position.y
            pos_arr[2] = p.position.z

            quat_arr[0] = p.orientation.x
            quat_arr[1] = p.orientation.y
            quat_arr[2] = p.orientation.z
            quat_arr[3] = p.orientation.w

            start = timer()
            xopt = lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))
            end = timer()
            print("Speed: {}".format(1.0 / (end - start)))

            ja = JointAngles()
            for i in range(xopt.length):
                ja.angles.data.append(xopt.data[i])
            angles_pub.publish(ja)
            
            # print(ja.angles.data)
            ja_stream.append(ja.angles.data)

            pose = kdl_kin.forward(ja.angles.data)
            trans_cur = [pose[0,3], pose[1,3], pose[2,3]]
            rot_cur = T.quaternion_from_matrix(pose)
            # print(trans, rot)

            trans_goal = numpy.array(init_trans) + numpy.array([p.position.x, p.position.y, p.position.z])
            rot_goal = T.quaternion_multiply([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z], init_rot)
            # print("Next goal position: {}\nNext goal orientation: {}".format(list(trans_goal), list(rot_goal)))
            
            dis = numpy.linalg.norm(numpy.array(trans_cur) - numpy.array(trans_goal))
            angle_between = numpy.linalg.norm(T.quaternion_disp(rot_cur, rot_goal)) * 2.0
            # print(dis, angle_between)

            if dis < pos_goal_tolerance and angle_between < quat_goal_tolerance and keyframe < len(waypoints) - 1 - step:
                keyframe += step

            if round(keyframe) >= len(waypoints) - 1: break

            rate.sleep()

        # print(ja_stream)
        print("\nSize of the joint state stream: {}".format(len(ja_stream)))
    
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
