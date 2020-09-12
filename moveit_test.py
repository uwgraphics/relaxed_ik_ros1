#! /usr/bin/env python

import cartesian_path
import ctypes
import moveit_commander
import moveit_msgs.msg
import os
import rospkg
import rospy
import sys
import transformations as T
import yaml

from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander.conversions import pose_to_list
from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float64, String
from timeit import default_timer as timer
from visualization_msgs.msg import InteractiveMarkerFeedback

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

def add_collision_object(scene, name, planning_frame, shape, trans, rots, scale, is_dynamic):
    p = PoseStamped()
    p.header.frame_id = planning_frame
    p.pose.position.x = trans[0]
    p.pose.position.y = trans[1]
    p.pose.position.z = trans[2]
    quat = T.quaternion_from_euler(rots[0], rots[1], rots[2])
    p.pose.orientation.w = quat[0]
    p.pose.orientation.x = quat[1]
    p.pose.orientation.y = quat[2]
    p.pose.orientation.z = quat[3]

    if shape == 'box':
        scene.add_box(name, p, (scale[0] * 2.0, scale[1] * 2.0, scale[2] * 2.0))
    elif shape == 'sphere':
        scene.add_sphere(name, p, scale[0])

    print(name)

def set_collision_world(robot, scene):
    planning_frame = robot.get_planning_frame()
    path_to_src = os.path.dirname(__file__)
    env_collision_file_path = path_to_src + '/env_collision_files/env_collision.yaml'
    if os.path.exists(env_collision_file_path):
        env_collision_file = open(env_collision_file_path, 'r')
        env_collision = yaml.load(env_collision_file)
        
        if 'boxes' in env_collision: 
            planes = env_collision['boxes']
            if planes is not None:
                for i, p in enumerate(planes):
                    add_collision_object(scene, p['name'], planning_frame, "box", p['translation'], p['rotation'], p['parameters'], p['is_dynamic'])

        if 'spheres' in env_collision:
            spheres = env_collision['spheres']
            if spheres is not None:
                for i, s in enumerate(spheres):
                    radius = s['parameters']
                    add_collision_object(scene, s['name'], planning_frame, "sphere", s['translation'], [0, 0, 0], [radius, radius, radius], s['is_dynamic'])
        
        # if 'point_cloud' in env_collision: 
        #     point_cloud = env_collision['point_cloud']
        #     if point_cloud is not None:
        #         for pc in point_cloud:
        #             scale = pc['scale']
        #             ts = pc['translation']
        #             rots = pc['rotation']

        #             header = Header()
        #             header.stamp = rospy.Time.now()
        #             header.frame_id = fixed_frame
        #             fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1), 
        #                 PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        #                 PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]

        #             pcd_path = path_to_src + '/env_collision_files/' + pc['file']
        #             points = []
        #             with open(pcd_path, 'r') as point_cloud_file:
        #                 lines = point_cloud_file.read().split('\n')
        #                 for line in lines:
        #                     pt = line.split(' ')
        #                     if is_point(pt):
        #                         pt_list = [float(pt[0]), float(pt[1]), float(pt[2]), 0.0]
        #                         rot_mat = T.euler_matrix(rots[0], rots[1], rots[2])
        #                         pt_rot = numpy.matmul(rot_mat, pt_list).tolist()
        #                         pt_new = [scale[0]*pt_rot[0]+ts[0], scale[1]*pt_rot[1]+ts[1], scale[2]*pt_rot[2]+ts[2]]
        #                         points.append(pt_new)

        #             points_msgs.append(pcl2.create_cloud(header, fields, points))

def main(args=None):
    print("\nMoveIt initialized!")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, dynObstacle_cb)
    # rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=3)
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #     moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(2)
    set_collision_world(robot, scene)
    print("============ Collision objects: {}".format(scene.get_objects()))

    move_group = moveit_commander.MoveGroupCommander("right_arm")
    move_group.set_end_effector_link("right_hand")

    print("============ Available Planning Groups: {}".format(robot.get_group_names()))
    print("============ Move group joints: {}".format(move_group.get_joints()))
    print("============ Planning frame: {}".format(move_group.get_planning_frame()))
    print("============ End effector link: {}".format(move_group.get_end_effector_link()))

    rospack = rospkg.RosPack()
    p = rospack.get_path('relaxed_ik_ros1') 
    relative_waypoints = cartesian_path.read_cartesian_path(p + "/cartesian_path_files/cartesian_path_prototype")
    waypoints = cartesian_path.get_abs_waypoints(relative_waypoints, move_group.get_current_pose().pose)

    # print(waypoints)

    # start = timer()
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    # end = timer()
    # print("Speed: {}".format(1.0 / (end - start)))

    # print(plan)

    # move_group.execute(plan)
    
    ja_stream = []
    for pt in plan.joint_trajectory.points[1:]:
        ja_stream.append(list(pt.positions))
    print("============ Size of the joint state stream: {}".format(len(ja_stream)))
    # print(ja_stream)

    rate = rospy.Rate(300)
    index = 0
    while not rospy.is_shutdown():
        ja = JointAngles()
        ja.angles.data = ja_stream[index]
        angles_pub.publish(ja)
        if index < len(ja_stream) - 1:
            index = index + 1
        rate.sleep()
    
    print("============ Waiting while RVIZ displays plan...")

if __name__ == '__main__':
    main()