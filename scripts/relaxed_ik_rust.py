#! /usr/bin/env python3

import ctypes
import numpy as np
import os
import rospkg
import rospy
import sys
import transformations as T
import yaml

from relaxed_ik_ros1.srv import IKPose, IKPoseResponse
from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState 
from urdf_parser_py.urdf import URDF
from kdl_parser import kdl_tree_from_urdf_model
import PyKDL as kdl
from robot import Robot

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

class RelaxedIKS(ctypes.Structure):
    pass

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'
lib = ctypes.cdll.LoadLibrary(path_to_src + '/target/debug/librelaxed_ik_lib.so')

lib.relaxed_ik_new.restype = ctypes.POINTER(RelaxedIKS)
lib.solve.argtypes = [ctypes.POINTER(RelaxedIKS), ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
lib.solve.restype = Opt
lib.solve_position.argtypes = [ctypes.POINTER(RelaxedIKS), ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
lib.solve_position.restype = Opt
lib.solve_velocity.argtypes = [ctypes.POINTER(RelaxedIKS), ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
lib.solve_velocity.restype = Opt

class RelaxedIKRust:
    def __init__(self, setting_file_path = None):
        if setting_file_path is None:
            self.obj = lib.relaxed_ik_new(ctypes.c_char_p())
        else:
            self.obj = lib.relaxed_ik_new(ctypes.c_char_p(setting_file_path.encode('utf-8')))
    
    def __exit__(self, exc_type, exc_value, traceback):
        lib.relaxed_ik_free(self.obj)
    
    def solve_position(self, positions, orientations, tolerances):
        pos_arr = (ctypes.c_double * len(positions))()
        quat_arr = (ctypes.c_double * len(orientations))()
        tole_arr = (ctypes.c_double * len(tolerances))()
        for i in range(len(positions)):
            pos_arr[i] = positions[i]
        for i in range(len(orientations)):
            quat_arr[i] = orientations[i]
        for i in range(len(tolerances)):
            tole_arr[i] = tolerances[i]
        xopt = lib.solve_position(self.obj, pos_arr, len(pos_arr), quat_arr, len(quat_arr), tole_arr, len(tole_arr))
        return xopt.data[:xopt.length]
    
    def solve_velocity(self, linear_velocities, angular_velocities, tolerances):
        linear_arr = (ctypes.c_double * len(linear_velocities))()
        angular_arr = (ctypes.c_double * len(angular_velocities))()
        tole_arr = (ctypes.c_double * len(tolerances))()
        for i in range(len(linear_velocities)):
            linear_arr[i] = linear_velocities[i]
        for i in range(len(angular_velocities)):
            angular_arr[i] = angular_velocities[i]
        for i in range(len(tolerances)):
            tole_arr[i] = tolerances[i]
        xopt = lib.solve_velocity(self.obj, linear_arr, len(linear_arr), angular_arr, len(angular_arr), tole_arr, len(tole_arr))
        return xopt.data[:xopt.length]

    def get_ee_positions(self):
        xopt = lib.get_ee_positions(self.obj)
        return xopt.data[:xopt.length]

class RelaxedIK:
    def __init__(self):
        deault_setting_file_path = path_to_src + '/configs/settings.yaml'

        setting_file_path = rospy.get_param('setting_file_path')
        if setting_file_path == '':
            print("Rviz viewer: no setting file path is given, using default setting files --" + setting_file_path)
            setting_file_path = deault_setting_file_path

        os.chdir(path_to_src )

        # Load the infomation
        setting_file = open(setting_file_path, 'r')
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)
       
        urdf_file = open(path_to_src + '/configs/urdfs/' + settings["urdf"], 'r')
        urdf_string = urdf_file.read()
        rospy.set_param('robot_description', urdf_string)

        self.relaxed_ik = RelaxedIKRust(setting_file_path)

        # Services
        self.ik_pose_service = rospy.Service('relaxed_ik/solve_pose', IKPose, self.handle_ik_pose)

        # Publishers
        self.angles_pub = rospy.Publisher('relaxed_ik/joint_angle_solutions', JointState, queue_size=1)
   
        self.robot = Robot(setting_file_path)

        self.js_msg = JointState()
        self.js_msg.name = self.robot.articulated_joint_names
        self.js_msg.position = []
        for i in range(len(self.js_msg.name)):
            self.js_msg.position.append( settings['starting_config'][i] )
        
        # Subscribers
        rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, self.pose_goals_cb)
        rospy.Subscriber('/relaxed_ik/ee_vel_goals', EEVelGoals, self.pose_vels_cb)
        rospy.Subscriber('/relaxed_ik/reset', JointState, self.reset_cb)

        print("\nSolver RelaxedIK initialized!\n")

    def get_ee_pose(self):
        ee_poses = self.relaxed_ik.get_ee_positions()
        ee_poses = np.array(ee_poses)
        ee_poses = ee_poses.reshape((len(ee_poses)//6, 6))
        ee_poses = ee_poses.tolist()
        return ee_poses

    def handle_ik_pose(self, req):
        positions = []
        orientations = []
        tolerances = []
        for i in range(len(req.ee_poses)):
            positions.append(req.ee_poses[i].position.x)
            positions.append(req.ee_poses[i].position.y)
            positions.append(req.ee_poses[i].position.z)
            orientations.append(req.ee_poses[i].orientation.x)
            orientations.append(req.ee_poses[i].orientation.y)
            orientations.append(req.ee_poses[i].orientation.z)
            orientations.append(req.ee_poses[i].orientation.w)
            if i > len(req.tolerances):
                tolerances.append(req.tolerances[i].linear.x)
                tolerances.append(req.tolerances[i].linear.y)
                tolerances.append(req.tolerances[i].linear.z)
                tolerances.append(req.tolerances[i].angular.x)
                tolerances.append(req.tolerances[i].angular.y)
                tolerances.append(req.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)
        res = IKPoseResponse()
        res.joint_state = ik_solution

        return res

    def reset_cb(self, msg):
        n = len(msg.positions)
        x = (ctypes.c_double * n)()
        for i in range(n):
            x[i] = msg.positions[i]
        self.relaxed_ik.reset(x, n)

    def pose_goals_cb(self, msg):
        positions = []
        orientations = []
        tolerances = []
        for i in range(len(msg.ee_poses)):
            positions.append(msg.ee_poses[i].position.x)
            positions.append(msg.ee_poses[i].position.y)
            positions.append(msg.ee_poses[i].position.z)
            orientations.append(msg.ee_poses[i].orientation.x)
            orientations.append(msg.ee_poses[i].orientation.y)
            orientations.append(msg.ee_poses[i].orientation.z)
            orientations.append(msg.ee_poses[i].orientation.w)
            if i > len(msg.tolerances):
                tolerances.append(msg.tolerances[i].linear.x)
                tolerances.append(msg.tolerances[i].linear.y)
                tolerances.append(msg.tolerances[i].linear.z)
                tolerances.append(msg.tolerances[i].angular.x)
                tolerances.append(msg.tolerances[i].angular.y)
                tolerances.append(msg.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        before = rospy.get_rostime()
        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)
        after = rospy.get_rostime()
        duration = after - before 

        # Publish the joint angle solution
        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)

    def pose_vels_cb(self, msg):
        linear_vels = []
        angular_vels = []
        tolerances = []
        for i in range(len(msg.ee_vels)):
            linear_vels.append(msg.ee_vels[i].linear.x)
            linear_vels.append(msg.ee_vels[i].linear.y)
            linear_vels.append(msg.ee_vels[i].linear.z)
            angular_vels.append(msg.ee_vels[i].angular.x)
            angular_vels.append(msg.ee_vels[i].angular.y)
            angular_vels.append(msg.ee_vels[i].angular.z)
            if i > len(msg.tolerances):
                tolerances.append(msg.tolerances[i].linear.x)
                tolerances.append(msg.tolerances[i].linear.y)
                tolerances.append(msg.tolerances[i].linear.z)
                tolerances.append(msg.tolerances[i].angular.x)
                tolerances.append(msg.tolerances[i].angular.y)
                tolerances.append(msg.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        before = rospy.get_rostime()
        ik_solution = self.relaxed_ik.solve_velocity(linear_vels, angular_vels, tolerances)
        after = rospy.get_rostime()
        duration = after - before 

        assert len(ik_solution) == len(self.robot.articulated_joint_names)

        # Publish the joint angle solution
        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)

if __name__ == '__main__':
    rospy.init_node('relaxed_ik')
    relaxed_ik = RelaxedIK()
    rospy.spin()
