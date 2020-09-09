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
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback
from timeit import default_timer as timer
import transformations as T
from cartesian_path import readCartesianPath

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
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("right_arm")
    move_group.set_end_effector_link("right_hand")

    print("============ Available Planning Groups: {}".format(robot.get_group_names()))
    print("============ Move group joints: {}".format(move_group.get_joints()))
    print("============ Planning frame: {}".format(move_group.get_planning_frame()))
    print("============ End effector link: {}".format(move_group.get_end_effector_link()))
    
    init_pose = move_group.get_current_pose()
    init_position = init_pose.pose.position
    init_orientation = [init_pose.pose.orientation.w, init_pose.pose.orientation.x, init_pose.pose.orientation.y, init_pose.pose.orientation.z]

    rospack = rospkg.RosPack()
    p = rospack.get_path('relaxed_ik_ros1') 
    relative_waypoints = readCartesianPath(p + "/cartesian_path_prototype")
    waypoints = []
    for relative_goal in relative_waypoints:
        relative_orientation_goal = [relative_goal.orientation.w, relative_goal.orientation.x, relative_goal.orientation.y, relative_goal.orientation.z]
        goal = Pose()
        goal.position.x = relative_goal.position.x + init_position.x
        goal.position.y = relative_goal.position.y + init_position.y
        goal.position.z = relative_goal.position.z + init_position.z

        goal_orientation = T.quaternion_multiply(relative_orientation_goal, init_orientation)
        goal.orientation.w = goal_orientation[0]
        goal.orientation.x = goal_orientation[1]
        goal.orientation.y = goal_orientation[2]
        goal.orientation.z = goal_orientation[3]
        waypoints.append(goal)

    # print(waypoints)

    # start = timer()
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan)
    # end = timer()
    # print("Speed: {}".format(1.0 / (end - start)))
    
    print("============ Waiting while RVIZ displays plan...")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

    # while eepg == None: continue

    # rate = rospy.Rate(3000)
    # while not rospy.is_shutdown():
    #     pose_goals = eepg.ee_poses
    #     header = eepg.header

    #     relative_goal = pose_goals[0]
    #     relative_orientation_goal = [relative_goal.orientation.w, relative_goal.orientation.x, relative_goal.orientation.y, relative_goal.orientation.z]
    #     goal = Pose()
    #     goal.position.x = relative_goal.position.x + init_position.x
    #     goal.position.y = relative_goal.position.y + init_position.y
    #     goal.position.z = relative_goal.position.z + init_position.z

    #     goal_orientation = T.quaternion_multiply(init_orientation, relative_orientation_goal)
    #     goal.orientation.w = goal_orientation[0]
    #     goal.orientation.x = goal_orientation[1]
    #     goal.orientation.y = goal_orientation[2]
    #     goal.orientation.z = goal_orientation[3]
        
    #     move_group.set_pose_target(goal, end_effector_link="right_hand")
    #     move_group.set_goal_tolerance(0.01)

    #     move_group.go(wait=True)
    #     print(move_group.get_current_pose())
    #     move_group.stop()
    #     move_group.clear_pose_targets()

    #     joint_values = move_group.get_current_joint_values()
    #     ja = JointAngles()
    #     ja.header = header
    #     for j in joint_values:
    #         ja.angles.data.append(j)

    #     angles_pub.publish(ja)
    #     print(joint_values)

    #     rate.sleep()

if __name__ == '__main__':
    main()