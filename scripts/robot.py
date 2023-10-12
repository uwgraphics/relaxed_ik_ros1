#! /usr/bin/env python3

import os
from timeit import default_timer as timer

import numpy as np
import PyKDL
import rospkg
import rospy
import transformations as T
import yaml
from geometry_msgs.msg import Pose, Vector3
from kdl_parser import kdl_tree_from_urdf_model
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String
from urdf_parser_py.urdf import URDF


class Robot:
    def __init__(self, setting_path=None):
        path_to_src = rospkg.RosPack().get_path("relaxed_ik_ros1") + "/relaxed_ik_core"
        setting_file_path = path_to_src + "/configs/settings.yaml"
        if setting_path != "":
            setting_file_path = setting_path
        os.chdir(path_to_src)

        # Load the infomation
        setting_file = open(setting_file_path, "r")
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)

        urdf_name = settings["urdf"]

        self.robot = URDF.from_xml_file(path_to_src + "/configs/urdfs/" + urdf_name)
        self.kdl_tree = kdl_tree_from_urdf_model(self.robot)

        # all non-fixed joint
        self.joint_lower_limits = []
        self.joint_upper_limits = []
        self.joint_vel_limits = []
        self.all_joint_names = []
        for j in self.robot.joints:
            if j.type != "fixed":
                self.joint_lower_limits.append(j.limit.lower)
                self.joint_upper_limits.append(j.limit.upper)
                self.joint_vel_limits.append(j.limit.velocity)
                self.all_joint_names.append(j.name)

        # joints solved by relaxed ik
        self.articulated_joint_names = []
        assert len(settings["base_links"]) == len(settings["ee_links"])
        self.num_chain = len(settings["base_links"])

        for i in range(self.num_chain):
            arm_chain = self.kdl_tree.getChain(
                settings["base_links"][i], settings["ee_links"][i]
            )
            for j in range(arm_chain.getNrOfSegments()):
                joint = arm_chain.getSegment(j).getJoint()
                # 8 is fixed joint
                if (
                    joint.getType() != 8
                    and joint.getName() not in self.articulated_joint_names
                ):
                    self.articulated_joint_names.append(joint.getName())

        if "joint_ordering" in settings and "starting_config" in settings:
            assert len(settings["joint_ordering"]) == len(
                settings["starting_config"]
            ), "Number of joints parsed from urdf should be the same with the starting config in the setting file."

        self.arm_chains = []
        self.chain_indices = []
        self.fk_p_kdls = []
        self.fk_v_kdls = []
        self.ik_p_kdls = []
        self.ik_v_kdls = []
        # self.num_jnts = []
        for i in range(self.num_chain):
            arm_chain = self.kdl_tree.getChain(
                settings["base_links"][i], settings["ee_links"][i]
            )
            self.arm_chains.append(arm_chain)
            self.fk_p_kdls.append(PyKDL.ChainFkSolverPos_recursive(arm_chain))
            self.fk_v_kdls.append(PyKDL.ChainFkSolverVel_recursive(arm_chain))
            self.ik_v_kdls.append(PyKDL.ChainIkSolverVel_pinv(arm_chain))
            self.ik_p_kdls.append(
                PyKDL.ChainIkSolverPos_NR(
                    arm_chain, self.fk_p_kdls[i], self.ik_v_kdls[i]
                )
            )
            joint_indices = []
            articulated_joint_index = 0
            for j in range(arm_chain.getNrOfSegments()):
                joint = arm_chain.getSegment(j).getJoint()
                if "joint_ordering" not in settings:
                    if joint.getType() != 8:
                        joint_indices.append(articulated_joint_index)
                        articulated_joint_index += 1
                else:
                    try:
                        joint_index = settings["joint_ordering"].index(joint.getName())
                        joint_indices.append(joint_index)
                    except ValueError:
                        pass

            self.chain_indices.append(joint_indices)
            # self.num_jnts.append(arm_chain.getNrOfJoints())

    def fk_single_chain(self, fk_p_kdl, joint_angles, num_jnts):
        assert (
            len(joint_angles) == num_jnts
        ), "length of input: {}, number of joints: {}".format(
            len(joint_angles), num_jnts
        )

        kdl_array = PyKDL.JntArray(num_jnts)
        for idx in range(num_jnts):
            kdl_array[idx] = joint_angles[idx]

        end_frame = PyKDL.Frame()
        fk_p_kdl.JntToCart(kdl_array, end_frame)

        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        return pose

    def fk(self, joint_angles):
        poses = []
        for i in range(self.num_chain):
            joint_values = [joint_angles[x] for x in self.chain_indices[i]]
            pose = self.fk_single_chain(
                self.fk_p_kdls[i],
                joint_values,
                len(self.chain_indices[i]),
            )
            poses.append(pose)

        return poses

    def get_joint_state_msg(self, joint_angles):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self._joint_names
        js.position = joint_angles
        return js
