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
from robot import Robot
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from std_msgs.msg import Float64, String
from timeit import default_timer as timer
from urdf_load import urdf_load
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

co_updates = []
def marker_feedback_cb(msg):
    global co_updates
    co_updates = []
    co_updates.append((msg.marker_name, msg.pose))

def marker_update_cb(msg):
    global co_updates
    co_updates = []
    for pose_stamped in msg.poses:
        co_updates.append((pose_stamped.name, pose_stamped.pose))

def add_collision_object(scene, name, planning_frame, shape, trans, rots, scale, is_dynamic, filename='', co_pub=None):
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
    # add the object to the scene based on its shape
    if shape == 'box':
        scene.add_box(name, p, (scale[0] * 2.0, scale[1] * 2.0, scale[2] * 2.0))
    elif shape == 'sphere':
        scene.add_sphere(name, p, scale[0])
    elif shape == 'pts' or shape == 'pcd':
        pcd = open3d.geometry.PointCloud()
        if shape == 'pts':
            point_cloud = numpy.loadtxt(filename, skiprow=1)
        else:
            point_cloud = numpy.loadtxt(filename, delimiter=',')
        # open3d.visualization.draw_geometries([pcd])
        pcd.points = open3d.utility.Vector3dVector(point_cloud[:,:3])
        convex_hull = pcd.compute_convex_hull()[0]
        # open3d.visualization.draw_geometries([convex_hull])
        # Create a collision object
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = p.header
        # Publish the mesh message
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
        co_pub.publish(co)
    elif shape == "mesh":
        scene.add_mesh(name, p, filename, size=(0.02 * scale[0], 0.02 * scale[0], 0.02 * scale[0]))
    else:
        print("Unrecognized shape: {}".format(shape))

def set_collision_world(robot, scene, co_pub, file_type='rmos'):
    planning_frame = robot.get_planning_frame()
    path_to_src = os.path.dirname(__file__)
    if file_type == 'rmos':
        env_collision_file_path = path_to_src + '/rmos_files/test.rmos'
        if os.path.exists(env_collision_file_path):
            with open(env_collision_file_path, 'r') as env_collision_file:
                lines = env_collision_file.read().split('\n')
                file_break = False
                for line in lines:
                    if len(line) == 0: continue
                    if line[0] == '#':
                        file_break = True
                        continue
                    if line[0].isalnum():
                        if not file_break:
                            continue
                    else:
                        continue
                    data_no_comment = line.split('//')
                    data = data_no_comment[0].strip().split(';')
                    name = data[0]
                    # scale = float(data[1])
                    motion_file = data[2]
                    is_dynamic = True
                    if motion_file == "static":
                        is_dynamic = False

                    pcd_path = path_to_src + '/point_cloud_files/' + name
                    add_collision_object(scene, name, planning_frame, "pcd", [0.0, 0.0, 0.0], \
                        [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], is_dynamic, filename=pcd_path, co_pub=co_pub)
    else:
        env_collision_file_path = path_to_src + '/env_collision_files/env_collision.yaml'
        if os.path.exists(env_collision_file_path):
            env_collision_file = open(env_collision_file_path, 'r')
            env_collision = yaml.load(env_collision_file)
            
            if 'boxes' in env_collision: 
                planes = env_collision['boxes']
                if planes is not None:
                    for i, p in enumerate(planes):
                        add_collision_object(scene, p['name'], planning_frame, "box", p['translation'], \
                            p['rotation'], p['parameters'], p['is_dynamic'])

            if 'spheres' in env_collision:
                spheres = env_collision['spheres']
                if spheres is not None:
                    for i, s in enumerate(spheres):
                        radius = s['parameters']
                        add_collision_object(scene, s['name'], planning_frame, "sphere", s['translation'], \
                            [0, 0, 0], [radius, radius, radius], s['is_dynamic'])
            
            if 'point_cloud' in env_collision: 
                point_cloud = env_collision['point_cloud']
                if point_cloud is not None:
                    for pc in point_cloud:
                        pcd_path = path_to_src + '/env_collision_files/' + pc['file']
                        add_collision_object(scene, pc['name'], planning_frame, "pts", pc['translation'], \
                            pc['rotation'], pc['scale'], pc['is_dynamic'], filename=pcd_path, co_pub=co_pub)

            if 'tri_mesh' in env_collision: 
                tri_mesh = env_collision['tri_mesh']
                if tri_mesh is not None:
                    for m in tri_mesh:
                        mesh_path = path_to_src + '/env_collision_files/' + m['file']
                        add_collision_object(scene, m['name'], planning_frame, "mesh", m['translation'], \
                            m['rotation'], m['parameters'], m['is_dynamic'], filename=mesh_path)

def main(args=None):
    print("\nMoveIt initialized!")

    # Global var to keep the update info of collision objects    
    global co_updates

    # Load relevant information
    path_to_src = os.path.dirname(__file__)
    info_file_name = open(path_to_src + '/relaxed_ik_core/config/loaded_robot', 'r').read()
    info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
    info_file = open(info_file_path, 'r')
    y = yaml.load(info_file)
    fixed_ee_joints = y['ee_fixed_joints']
    ee_links = test_utils.get_ee_link(info_file_name)
    if ee_links == None: ee_links = fixed_ee_joints
    full_joint_lists = y['joint_names']
    joint_order = y['joint_ordering']
    num_chains = len(full_joint_lists)
    group_name = test_utils.get_group_name(info_file_name)

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot_commander = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # Initialize Move group interface
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_end_effector_link(ee_links[0])
    # move_group.allow_replanning(True)

    # Publishers
    time_pub = rospy.Publisher('/relaxed_ik/current_time', Float64, queue_size=10)
    co_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)

    # Subscribers
    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb)
    rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb)
    
    # Services
    service_timeout = 5.0
    planning_scene_srv = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    planning_scene_srv.wait_for_service(service_timeout)
    state_validity_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
    state_validity_srv.wait_for_service(service_timeout)

    # Set up Relaxed IK Python robot
    arms = []
    for i in range(num_chains):
        urdf_robot, arm, arm_c, tree = urdf_load('', '', '', full_joint_lists[i], fixed_ee_joints[i])
        arms.append(arm)
    robot = Robot(arms, full_joint_lists, joint_order)
    
    # Set up the collision world
    rospy.sleep(2)
    set_collision_world(robot_commander, scene, co_pub)

    # Print help messages
    print("============ Collision objects: {}".format(scene.get_known_object_names()))
    print("============ Available Planning Groups: {}".format(robot_commander.get_group_names()))
    print("============ Move group joints: {}".format(move_group.get_joints()))
    print("============ Planning frame: {}".format(move_group.get_planning_frame()))
    print("============ End effector link: {}".format(move_group.get_end_effector_link()))

    # Read the cartesian path
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('relaxed_ik_ros1') 
    env_collision_file_path = path_to_src + '/rmos_files/test.rmos'
    if os.path.exists(env_collision_file_path):
        with open(env_collision_file_path, 'r') as env_collision_file:
            lines = env_collision_file.read().split('\n')
            data_no_comment = lines[0].split('//')
            data = data_no_comment[0].strip().split(';')
            cartesian_path_file_name = data[-1]
    relative_waypoints = test_utils.read_cartesian_path(package_path + "/cartesian_path_files/" + cartesian_path_file_name, scale=1.0)
    init_pose = move_group.get_current_pose().pose
    waypoints = test_utils.get_abs_waypoints(relative_waypoints, init_pose)
    final_trans_goal = [waypoints[-1][1].position.x, waypoints[-1][1].position.y, waypoints[-1][1].position.z]
    final_rot_goal = [waypoints[-1][1].orientation.w, waypoints[-1][1].orientation.x, waypoints[-1][1].orientation.y, waypoints[-1][1].orientation.z]

    # Wait for the start signal
    print("Waiting for ROS param /exp_status to be set as go...")
    initialized = False
    while not initialized: 
        try: 
            param = rospy.get_param("exp_status")
            initialized = param == "go"
        except KeyError:
            initialized = False

    # Set up initial params
    goal_idx = 1
    cur_time = 0.0
    delta_time = 0.01
    max_time = len(waypoints) * delta_time * 5.0
    num_collisions = 0
    pos_goal_tolerance = 0.01
    quat_goal_tolerance = 0.01
    interpolation_times = 3

    # Calculate the initial plan
    (time, p) = test_utils.linear_interpolate_waypoints(waypoints, goal_idx)
    move_group.set_pose_target(p)
    plan = move_group.plan()
    trajectory = [list(plan.joint_trajectory.points[x].positions) for x in range(len(plan.joint_trajectory.points))]
    trajectory = test_utils.linear_interpolate_joint_states(trajectory, interpolation_times)
    ja_list = list(list(trajectory[0]))
    if info_file_name == "hubo8_info.yaml":
        del ja_list[-2]
    ja_stream = [ja_list]

    # Start the calculation
    rate = rospy.Rate(3000)
    while not rospy.is_shutdown():
        if cur_time >= max_time: break

        # Publish the current time
        cur_time_msg = Float64()
        cur_time_msg.data = cur_time
        time_pub.publish(cur_time_msg)

        # Check for updates of collision objects
        if len(co_updates) > 0:
            ps = PlanningScene()
            ps.is_diff = True
            ps.robot_state.is_diff = True
            for name, pose in co_updates:
                old_co = scene.get_objects([name])[name]
                co = deepcopy(old_co)
                co.operation = CollisionObject.MOVE
                if len(co.primitives) > 0:
                    co.primitive_poses = [pose]
                else:
                    co.mesh_poses = [pose]
                ps.world.collision_objects.append(co)
            # Send the diff request
            diff_req = ApplyPlanningSceneRequest()
            diff_req.scene = ps
            planning_scene_srv.call(diff_req)
            # Clear the updates
            co_updates = []
        
        if len(trajectory) == 0:
            ja_stream.append(ja_stream[-1])
            cur_time += delta_time
            if goal_idx < len(waypoints) - 1:
                goal_idx += 1

        # Check collision and execuate the plan returned by MoveIt
        # print("goal index: {}, cur time: {}".format(goal_idx, cur_time))
        in_collision = False
        for i, traj_point in enumerate(trajectory[1:]):
            # Check for collision
            rs = RobotState()
            rs.joint_state.name = plan.joint_trajectory.joint_names
            rs.joint_state.position = traj_point
            sv_req = GetStateValidityRequest()
            sv_req.robot_state = rs
            sv_req.group_name = group_name
            result = state_validity_srv.call(sv_req)
            # if result is not valid, it is in collision
            if not result.valid:
                num_collisions += 1
                # print("Collision occured at trajectory point {}!".format(traj_point))
                in_collision = True
                plan_partial = deepcopy(plan)
                plan_partial.joint_trajectory.points = plan.joint_trajectory.points[:i]
                move_group.execute(plan_partial)
                # Trigger a replan
                (time, p) = test_utils.linear_interpolate_waypoints(waypoints, goal_idx)
                move_group.set_pose_target(p)
                plan = move_group.plan()

                ja_stream.append(ja_stream[-1])
                cur_time += delta_time
                if goal_idx < len(waypoints) - 1:
                    goal_idx += 1
                break
            else:
                ja_list = list(traj_point)
                if info_file_name == "hubo8_info.yaml":
                    del ja_list[-2]
                ja_stream.append(ja_list)

                ja = JointAngles()
                ja.angles.data = ja_list
                angles_pub.publish(ja)

                cur_time += delta_time
                if goal_idx < len(waypoints) - 1:
                    goal_idx += 1

        trans_cur = robot.get_ee_positions(ja_stream[-1])[0]
        rot_cur = robot.get_ee_rotations(ja_stream[-1])[0]
        dis = numpy.linalg.norm(numpy.array(trans_cur) - numpy.array(final_trans_goal))
        angle_between = numpy.linalg.norm(T.quaternion_disp(rot_cur, final_rot_goal)) * 2.0
        
        if dis < pos_goal_tolerance and angle_between < quat_goal_tolerance and cur_time > len(waypoints) * delta_time: 
            print("The path is finished successfully")
            break
        
        if not in_collision:
            move_group.execute(plan)
            (time, p) = test_utils.linear_interpolate_waypoints(waypoints, goal_idx)
            move_group.set_pose_target(p)
            plan = move_group.plan()
            trajectory = [list(plan.joint_trajectory.points[x].positions) for x in range(len(plan.joint_trajectory.points))]
            trajectory = test_utils.linear_interpolate_joint_states(trajectory, interpolation_times)

        rate.sleep()

    robot_name = info_file_name.split('_')[0]
    benchmark_evaluator = test_utils.BenchmarkEvaluator(relative_waypoints, ja_stream, delta_time, 1, \
            package_path + "/rmoo_files", "moveit", robot_name)
    benchmark_evaluator.write_ja_stream(interpolate=False)
    v_avg, a_avg, jerk_avg = benchmark_evaluator.calculate_joint_stats()
    pos_error_avg, rot_error_avg = benchmark_evaluator.calculate_error_stats()

    robot_str = "Robot: {}\n".format(robot_name)
    software_str = "Software: MoveIt!\n"
    motion_time_str = "Planned motion length: {}\nActual motion length: {}\n".format(len(waypoints) * delta_time, cur_time)
    joint_stream_str = "Size of the joint state stream: {}\n".format(len(ja_stream))
    joint_stats_str = "Average joint velocity: {}\nAverage joint acceleration: {}\nAverage joint jerk: {}\n"\
        .format(v_avg, a_avg, jerk_avg)
    err_stats_str = "Average position error: {}\nAverage rotation error: {}\n".format(pos_error_avg, rot_error_avg)
    num_collisions_str = "Number of environment collisions: {}".format(num_collisions)
    print(robot_str + software_str + motion_time_str + joint_stream_str + joint_stats_str + err_stats_str + num_collisions_str)

    rmob_file_path = package_path + '/rmob_files/' + robot_name + '/' + robot_name + '_moveit.rmob'
    with open(rmob_file_path, 'w') as rmob_file:
        rmob_file.write(robot_str + software_str + motion_time_str + joint_stream_str + joint_stats_str + err_stats_str + num_collisions_str)

if __name__ == '__main__':
    main()