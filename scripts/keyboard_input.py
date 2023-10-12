#!/usr/bin/python3

import numpy as np
import readchar
import rospkg
import rospy
import transformations as T
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    QuaternionStamped,
    Twist,
    Vector3Stamped,
)
from pynput import keyboard
from robot import Robot
from std_msgs.msg import Bool

from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals

path_to_src = rospkg.RosPack().get_path("relaxed_ik_ros1") + "/relaxed_ik_core"


class KeyboardInput:
    def __init__(self):
        deault_setting_file_path = path_to_src + "/configs/settings.yaml"

        setting_file_path = rospy.get_param("setting_file_path")
        if setting_file_path == "":
            setting_file_path = deault_setting_file_path

        self.robot = Robot(setting_file_path)

        self.ee_vel_goals_pub = rospy.Publisher(
            "relaxed_ik/ee_vel_goals", EEVelGoals, queue_size=5
        )

        self.pos_stride = 0.001
        self.rot_stride = 0.005

        self.seq = 1

        self.linear = np.zeros((self.robot.num_chain, 3))
        self.angular = np.zeros((self.robot.num_chain, 3))
        self.active_chain = 0

        keyboard_listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )

        rospy.Timer(rospy.Duration(0.033), self.timer_callback)

        keyboard_listener.start()

    def on_press(self, key):
        self.linear = np.zeros((self.robot.num_chain, 3))
        self.angular = np.zeros((self.robot.num_chain, 3))

        if key.char.isnumeric():
            assert int(key.char) <= self.robot.num_chain
            self.active_chain = int(key.char) - 1
        elif key.char == "w":
            self.linear[self.active_chain][0] += self.pos_stride
        elif key.char == "x":
            self.linear[self.active_chain][0] -= self.pos_stride
        elif key.char == "a":
            self.linear[self.active_chain][1] += self.pos_stride
        elif key.char == "d":
            self.linear[self.active_chain][1] -= self.pos_stride
        elif key.char == "e":
            self.linear[self.active_chain][2] += self.pos_stride
        elif key.char == "c":
            self.linear[self.active_chain][2] -= self.pos_stride
        elif key.char == "r":
            self.angular[self.active_chain][0] += self.rot_stride
        elif key.char == "v":
            self.angular[self.active_chain][0] -= self.rot_stride
        elif key.char == "d":
            self.angular[self.active_chain][1] += self.rot_stride
        elif key.char == "g":
            self.angular[self.active_chain][1] -= self.rot_stride
        elif key.char == "t":
            self.angular[self.active_chain][2] += self.rot_stride
        elif key.char == "b":
            self.angular[self.active_chain][2] -= self.rot_stride

        print(
            f"Chain: {self.active_chain+1}, Linear Vel: {self.linear[self.active_chain]}, Angular Vel: {self.angular[self.active_chain]}"
        )

    def on_release(self, key):
        self.linear = np.zeros((self.robot.num_chain, 3))
        self.angular = np.zeros((self.robot.num_chain, 3))

    def timer_callback(self, event):
        msg = EEVelGoals()

        for i in range(self.robot.num_chain):
            twist = Twist()
            twist.linear.x = self.linear[i][0]
            twist.linear.y = self.linear[i][1]
            twist.linear.z = self.linear[i][2]

            twist.angular.x = self.angular[i][0]
            twist.angular.y = self.angular[i][1]
            twist.angular.z = self.angular[i][2]

            tolerance = Twist()
            tolerance.linear.x = 0.0
            tolerance.linear.y = 0.0
            tolerance.linear.z = 0.0
            tolerance.angular.x = 0.0
            tolerance.angular.y = 0.0
            tolerance.angular.z = 0.0

            msg.ee_vels.append(twist)
            msg.tolerances.append(tolerance)

        self.seq += 1
        self.ee_vel_goals_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("keyboard_input")

    keyboard = KeyboardInput()
    rospy.spin()
