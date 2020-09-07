#! /usr/bin/env python

import rospy
import os
import rospkg
import ctypes
from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float64
from timeit import default_timer as timer
from visualization_msgs.msg import InteractiveMarkerFeedback

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

rospack = rospkg.RosPack()
p = rospack.get_path('relaxed_ik_ros1')
os.chdir(p + "/relaxed_ik_core")

lib = ctypes.cdll.LoadLibrary(p + '/relaxed_ik_core/target/debug/librelaxed_ik_lib.so')
lib.solve.restype = Opt

def dynObstacle_cb(msg):
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

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    global eepg

    print("\nSolver initialized!")

    rospy.init_node('relaxed_ik')

    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, dynObstacle_cb)

    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)

    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=3)

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
