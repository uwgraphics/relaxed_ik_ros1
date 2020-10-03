#! /usr/bin/env python

import ctypes
import moveit_commander
import numpy
import open3d
import os
import rospkg
import rospy
import sys
import test_utils
import transformations as T
import yaml

from copy import deepcopy
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest,\
                            GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from std_msgs.msg import Float64, String
from timeit import default_timer as timer
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

co_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)
service_timeout = 5.0
planning_scene_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
planning_scene_service.wait_for_service(service_timeout)

def marker_feedback_cb(msg, scene):
    # update dynamic collision obstacles in moveit
    co = scene.get_objects([msg.marker_name])[msg.marker_name]
    update_collision_object([co], [msg.pose])

def marker_update_cb(msg, scene):
    # update dynamic collision obstacles in moveit
    collison_objects = []
    poses = []
    for pose_stamped in msg.poses:
        collison_objects.append(scene.get_objects([pose_stamped.name])[pose_stamped.name])
        poses.append(pose_stamped.pose)
    update_collision_object(collison_objects, poses)

def add_collision_object(scene, name, planning_frame, shape, trans, rots, scale, is_dynamic, filename=''):
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
    elif shape == 'pcd':
        point_cloud = numpy.loadtxt(filename, skiprows=1)
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(point_cloud[:,:3])
        # open3d.visualization.draw_geometries([pcd])
        convex_hull = pcd.compute_convex_hull()[0]
        # open3d.visualization.draw_geometries([convex_hull])

        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = p.header

        mesh = Mesh()
        for face in convex_hull.triangles:
            triangle = MeshTriangle()
            triangle.vertex_indices = [face[0], face[1], face[2]]
            mesh.triangles.append(triangle)

        for vertex in convex_hull.vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        
        co.meshes = [mesh]
        co.mesh_poses = [p.pose]
        submit([co], False)
    elif shape == "mesh":
        scene.add_mesh(name, p, filename, size=(0.02 * scale[0], 0.02 * scale[0], 0.02 * scale[0]))

    print(name)

def update_collision_object(collison_objects, new_poses, synchronous=True):
    new_co = []
    for i, old_co in enumerate(collison_objects):
        co = deepcopy(old_co)
        co.operation = CollisionObject.MOVE
        if len(co.primitives) > 0:
            co.primitive_poses = [new_poses[i]]
        else:
            co.mesh_poses = [new_poses[i]]
        new_co.append(co)

    submit(new_co, synchronous)

def submit(new_co, synchronous):
    # print(new_co)
    if synchronous:
        scene = PlanningScene()
        scene.is_diff = True
        # scene.robot_state.is_diff = True
        scene.world.collision_objects = new_co
        diff_req = ApplyPlanningSceneRequest()
        diff_req.scene = scene
        planning_scene_service.call(diff_req)
    else:
        for co in new_co:
            co_pub.publish(co)

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
        
        if 'point_cloud' in env_collision: 
            point_cloud = env_collision['point_cloud']
            if point_cloud is not None:
                for pc in point_cloud:
                    pcd_path = path_to_src + '/env_collision_files/' + pc['file']
                    add_collision_object(scene, pc['name'], planning_frame, "pcd", pc['translation'], pc['rotation'], pc['scale'], pc['is_dynamic'], filename=pcd_path)

        if 'tri_mesh' in env_collision: 
            tri_mesh = env_collision['tri_mesh']
            if tri_mesh is not None:
                for m in tri_mesh:
                    mesh_path = path_to_src + '/env_collision_files/' + m['file']
                    add_collision_object(scene, m['name'], planning_frame, "mesh", m['translation'], m['rotation'], m['parameters'], m['is_dynamic'], filename=mesh_path)

def main(args=None):
    print("\nMoveIt initialized!")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=3)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_end_effector_link("right_hand")
    # move_group.allow_replanning(True)

    rospy.sleep(2)
    set_collision_world(robot, scene)

    # print("============ Collision objects: {}".format(scene.get_objects()))
    print("============ Available Planning Groups: {}".format(robot.get_group_names()))
    print("============ Move group joints: {}".format(move_group.get_joints()))
    print("============ Planning frame: {}".format(move_group.get_planning_frame()))
    print("============ End effector link: {}".format(move_group.get_end_effector_link()))

    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb, scene)
    rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb, scene)

    rospack = rospkg.RosPack()
    p = rospack.get_path('relaxed_ik_ros1') 
    relative_waypoints = test_utils.read_cartesian_path(p + "/cartesian_path_files/cartesian_path_prototype")
    init_pose = move_group.get_current_pose().pose
    waypoints = test_utils.get_abs_waypoints(relative_waypoints, init_pose)

    time_pub = rospy.Publisher('/relaxed_ik/current_time', Float64, queue_size=3)

    sv_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
    sv_srv.wait_for_service(service_timeout)
    
    initialized = False
    while not initialized: 
        try: 
            param = rospy.get_param("exp_status")
            initialized = param == "go"
        except KeyError:
            initialized = False

    ja_stream = []
    goal_idx = 1
    cur_time = 0.0
    delta_time = 0.01
    max_time = len(waypoints) * delta_time * 1000.0

    (time, p) = test_utils.linear_interpolate_waypoints(waypoints, goal_idx)
    move_group.set_pose_target(p)
    plan = move_group.plan()
    ja_stream.append(list(plan.joint_trajectory.points[0].positions))

    rate = rospy.Rate(3000)
    while not rospy.is_shutdown():
        if cur_time >= max_time: break

        cur_time_msg = Float64()
        cur_time_msg.data = cur_time
        time_pub.publish(cur_time_msg)
        
        # print("goal index: {}, cur time: {}".format(goal_idx, cur_time))
        in_collision = False
        for i, traj_point in enumerate(plan.joint_trajectory.points[1:]):
            rs = RobotState()
            rs.joint_state.name = plan.joint_trajectory.joint_names
            rs.joint_state.position = traj_point.positions
            gsvr = GetStateValidityRequest()
            gsvr.robot_state = rs
            gsvr.group_name = group_name
            result = sv_srv.call(gsvr)
            # if result is not valid, it is in collision
            if not result.valid:
                # print("Collision!")
                in_collision = True
                plan_partial = deepcopy(plan)
                plan_partial.joint_trajectory.points = plan.joint_trajectory.points[:i]
                move_group.execute(plan_partial)
                (time, p) = test_utils.linear_interpolate_waypoints(waypoints, goal_idx)
                move_group.set_pose_target(p)
                plan = move_group.plan()
                break
            else:
                ja_list = list(traj_point.positions)
                # print (ja_list)
                ja_stream.append(ja_list)

                ja = JointAngles()
                ja.angles.data = ja_list
                angles_pub.publish(ja)

                cur_time += delta_time
                goal_idx += 1

        if goal_idx > len(waypoints) - 1:
            break
        
        if not in_collision:
            move_group.execute(plan)
            (time, p) = test_utils.linear_interpolate_waypoints(waypoints, goal_idx)
            move_group.set_pose_target(p)
            plan = move_group.plan()

        rate.sleep()

    print("\nThe path is planned to take {} seconds and in practice it takes {} seconds".format(len(waypoints) * delta_time, cur_time))
    print("============ Size of the joint state stream: {}".format(len(ja_stream)))

if __name__ == '__main__':
    main()