#! /usr/bin/env python

import rospy
import os
import rospkg
import ctypes
from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float64

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

rospack = rospkg.RosPack()
p = rospack.get_path('relaxed_ik_ros1')
os.chdir(p + "/relaxed_ik_core")

lib = ctypes.cdll.LoadLibrary(p + '/relaxed_ik_core/target/debug/librelaxed_ik.so')
lib.run_ros1.restype = Opt

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    global eepg

    print("\nSolver initialized!\n")

    rospy.init_node('relaxed_ik')
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

            quat_arr[3*i] = p.orientation.w
            quat_arr[3*i+1] = p.orientation.x
            quat_arr[3*i+2] = p.orientation.y
            quat_arr[3*i+3] = p.orientation.z

        xopt = lib.run_ros1(pos_arr, len(pos_arr), quat_arr, len(quat_arr))

        ja = JointAngles()
        ja.header = header
        for i in range(xopt.length):
            ja.angles.data.append(xopt.data[i])

        angles_pub.publish(ja)

        rate.sleep()

if __name__ == '__main__':
    main()