#! /usr/bin/env python

import rospy
import sys
import os
import rospkg
import ctypes
import moveit_commander
from moveit_commander.conversions import pose_to_list
from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
import moveit_msgs.msg
from std_msgs.msg import Float64, String
import geometry_msgs.msg
from visualization_msgs.msg import InteractiveMarkerFeedback
from timeit import default_timer as timer

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

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    global eepg

    print("\nMoveIt initialized!")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, dynObstacle_cb)
    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)
    # angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=3)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # We can get a list of all the groups in the robot:
    print("============ Available Planning Groups:", robot.get_group_names())

    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    while eepg == None: continue

    rate = rospy.Rate(3000)
    while not rospy.is_shutdown():
        pose_goals = eepg.ee_poses
        header = eepg.header

        for i in range(len(pose_goals)):
            p = pose_goals[i]
            move_group.set_pose_target(p)

        # start = timer()
        plan = move_group.go(wait=True)
        # end = timer()
        # print("Speed: {}".format(1.0 / (end - start)))

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        # ja = JointAngles()
        # ja.header = header
        # ja_str = "["
        # for i in range(xopt.length):
        #     ja.angles.data.append(xopt.data[i])
        #     ja_str += str(xopt.data[i])
        #     if i == xopt.length - 1:
        #         ja_str += "]"
        #     else: 
        #         ja_str += ", "

        # angles_pub.publish(ja)
        # print(ja_str)

        rate.sleep()

if __name__ == '__main__':
    main()