#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/8/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''

import numpy
import os
import roslaunch
import rospkg
import rospy
import test_utils
import tf
import transformations as T
import yaml

from std_msgs.msg import ColorRGBA, Float64
from geometry_msgs.msg import Point
from interactive_markers.interactive_marker_server import *
from relaxed_ik_ros1.msg import JointAngles
from sensor_msgs.msg import JointState, PointCloud2, PointField
from timeit import default_timer as timer
from visualization_msgs.msg import *

ja_solution = ''
def ja_solution_cb(data):
    global ja_solution
    ja_solution = []
    for a in data.angles.data:
        ja_solution.append(a)

cur_time = 0.0
def time_update_cb(msg):
    global cur_time
    cur_time = msg.data
    print("The current time is {}".format(cur_time))

# def marker_feedback_cb(msg, args):
#     server = args
#     server.setPose(msg.marker_name, msg.pose)    
#     server.applyChanges()

def processFeedback(feedback):
    p = feedback.pose.position
    print (feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

def makeMarker(name, fixed_frame, shape, ts, rots, scale, is_dynamic, points=None, mesh_file=None):                
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = fixed_frame
    int_marker.name = name
    int_marker.pose.position.x = ts[0]
    int_marker.pose.position.y = ts[1]
    int_marker.pose.position.z = ts[2]

    quat = T.quaternion_from_euler(rots[0], rots[1], rots[2])
    int_marker.pose.orientation.x = quat[1]
    int_marker.pose.orientation.y = quat[2]
    int_marker.pose.orientation.z = quat[3]
    int_marker.pose.orientation.w = quat[0]

    int_marker.scale = 0.3

    marker = Marker()
    marker.scale.x = scale[0] * 2
    marker.scale.y = scale[1] * 2
    marker.scale.z = scale[2] * 2
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    if shape == "box":
        marker.type = Marker.CUBE
    elif shape == "sphere":
        marker.type = Marker.SPHERE
    elif shape == "pcd":
        marker.type = Marker.POINTS
        marker.points = points
    elif shape == "mesh":
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = mesh_file

    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(marker)
    int_marker.controls.append(control)

    if is_dynamic:
        c = 1.0 / numpy.sqrt(2)
        tx_control = InteractiveMarkerControl()
        tx_control.orientation.w = c
        tx_control.orientation.x = c
        tx_control.orientation.y = 0
        tx_control.orientation.z = 0
        tx_control.name = "move_x"
        tx_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(tx_control)

        mz_control = InteractiveMarkerControl()
        mz_control.orientation.w = c
        mz_control.orientation.x = 0
        mz_control.orientation.y = c
        mz_control.orientation.z = 0
        mz_control.name = "move_z"
        mz_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(mz_control)

        my_control = InteractiveMarkerControl()
        my_control.orientation.w = c
        my_control.orientation.x = 0
        my_control.orientation.y = 0
        my_control.orientation.z = c
        my_control.name = "move_y"
        my_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(my_control)

        if shape != "sphere":
            rx_control = InteractiveMarkerControl()
            rx_control.orientation.w = c
            rx_control.orientation.x = c
            rx_control.orientation.y = 0
            rx_control.orientation.z = 0
            rx_control.name = "rotate_x"
            rx_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(rx_control)

            rz_control = InteractiveMarkerControl()
            rz_control.orientation.w = c
            rz_control.orientation.x = 0
            rz_control.orientation.y = c
            rz_control.orientation.z = 0
            rz_control.name = "rotate_z"
            rz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(rz_control)

            ry_control = InteractiveMarkerControl()
            ry_control.orientation.w = c
            ry_control.orientation.x = 0
            ry_control.orientation.y = 0
            ry_control.orientation.z = c
            ry_control.name = "rotate_y"
            ry_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(ry_control)

    return int_marker

def set_collision_world(server, path_to_src, fixed_frame, file_type='rmos'):
    cartesian_folder_path = rospkg.RosPack().get_path('relaxed_ik_ros1') + "/cartesian_path_files/"
    if file_type == 'rmos':
        env_collision_file_path = path_to_src + '/rmos_files/test.rmos'
        if os.path.exists(env_collision_file_path):
            dyn_obstacle_handles = []
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
                    points = []
                    with open(pcd_path, 'r') as point_cloud_file:
                        point_lines = point_cloud_file.read().split('\n')
                        for point_line in point_lines:
                            pt = point_line.split(',')
                            if test_utils.is_point(pt):
                                point = Point()
                                point.x = float(pt[0])
                                point.y = float(pt[1])
                                point.z = float(pt[2])
                                # print(point)
                                points.append(point)
                        
                        int_marker = makeMarker(name, fixed_frame, "pcd", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.01, 0.01, 0.0], is_dynamic, points=points)
                        server.insert(int_marker, processFeedback)
                        if is_dynamic:
                            path = cartesian_folder_path + motion_file
                            relative_waypoints = test_utils.read_cartesian_path(path)
                            waypoints = test_utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                            dyn_obstacle_handles.append((int_marker.name, waypoints))

            server.applyChanges()

            return dyn_obstacle_handles
    else:
        env_collision_file_path = path_to_src + '/env_collision_files/env_collision.yaml'
        if os.path.exists(env_collision_file_path):
            env_collision_file = open(env_collision_file_path, 'r')
            env_collision = yaml.load(env_collision_file)
            
            dyn_obstacle_handles = []

            if 'boxes' in env_collision: 
                planes = env_collision['boxes']
                if planes is not None:
                    for p in planes:
                        int_marker = makeMarker(p['name'], fixed_frame, "box", p['translation'], p['rotation'], p['parameters'], p['is_dynamic'])
                        server.insert(int_marker, processFeedback)
                        if 'cartesian_path' in p and p['cartesian_path'] is not None:
                            path = cartesian_folder_path + p['cartesian_path']
                            relative_waypoints = test_utils.read_cartesian_path(path)
                            waypoints = test_utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                            dyn_obstacle_handles.append((int_marker.name, waypoints))

            if 'spheres' in env_collision:
                spheres = env_collision['spheres']
                if spheres is not None:
                    for s in spheres:
                        radius = s['parameters']
                        int_marker = makeMarker(s['name'], fixed_frame, "sphere", s['translation'], [0, 0, 0], [radius, radius, radius], s['is_dynamic'])
                        server.insert(int_marker, processFeedback)
                        if 'cartesian_path' in s and s['cartesian_path'] is not None:
                            path = cartesian_folder_path + s['cartesian_path']
                            relative_waypoints = test_utils.read_cartesian_path(path)
                            waypoints = test_utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                            dyn_obstacle_handles.append((int_marker.name, waypoints))

            if 'point_cloud' in env_collision: 
                point_cloud = env_collision['point_cloud']
                if point_cloud is not None:
                    for pc in point_cloud:
                        pcd_path = path_to_src + '/env_collision_files/' + pc['file']
                        scales = pc['scale']
                        points = []
                        with open(pcd_path, 'r') as point_cloud_file:
                            lines = point_cloud_file.read().split('\n')
                            for line in lines:
                                pt = line.split(' ')
                                if test_utils.is_point(pt):
                                    point = Point()
                                    point.x = float(pt[0]) * scales[0]
                                    point.y = float(pt[1]) * scales[1]
                                    point.z = float(pt[2]) * scales[2]
                                    points.append(point)
                        
                        int_marker = makeMarker(pc['name'], fixed_frame, "pcd", pc['translation'], pc['rotation'], [0.01, 0.01, 0.0], pc['is_dynamic'], points=points)
                        server.insert(int_marker, processFeedback)
                        if 'cartesian_path' in pc and pc['cartesian_path'] is not None:
                            path = cartesian_folder_path + pc['cartesian_path']
                            relative_waypoints = test_utils.read_cartesian_path(path)
                            waypoints = test_utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                            dyn_obstacle_handles.append((int_marker.name, waypoints))
            
            if 'tri_mesh' in env_collision:
                tri_meshes = env_collision['tri_mesh']
                if tri_meshes is not None:
                    for m in tri_meshes:
                        mesh_path = 'package://relaxed_ik_ros1/env_collision_files/' + m['file']
                        int_marker = makeMarker(m['name'], fixed_frame, "mesh", m['translation'], m['rotation'], m['parameters'], m['is_dynamic'], mesh_file=mesh_path)
                        server.insert(int_marker, processFeedback)
                        if 'cartesian_path' in m and m['cartesian_path'] is not None:
                            path = cartesian_folder_path + m['cartesian_path']
                            relative_waypoints = test_utils.read_cartesian_path(path)
                            waypoints = test_utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                            dyn_obstacle_handles.append((int_marker.name, waypoints))

            server.applyChanges()

            return dyn_obstacle_handles

def main(args=None):
    rospy.init_node('rviz_viewer')

    path_to_src = os.path.dirname(__file__)

    info_file_name = open(path_to_src + '/relaxed_ik_core/config/loaded_robot', 'r').read()
    info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
    info_file = open(info_file_path, 'r')

    y = yaml.load(info_file)
    urdf_file_name = y['urdf_file_name']
    fixed_frame = y['fixed_frame']
    joint_ordering = y['joint_ordering']
    starting_config = y['starting_config']
    joint_state_define_file_name = y['joint_state_define_func_file']
    joint_state_define_file = open(path_to_src + '/relaxed_ik_core/config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
    joint_state_define = joint_state_define_file.read()
    exec(joint_state_define)

    urdf_file = open(path_to_src + '/relaxed_ik_core/config/urdfs/' + urdf_file_name, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    js_pub = rospy.Publisher('joint_states',JointState,queue_size=5)
    rospy.Subscriber('/relaxed_ik/joint_angle_solutions',JointAngles,ja_solution_cb)
    tf_pub = tf.TransformBroadcaster()

    rospy.sleep(0.5)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = path_to_src + '/launch/joint_state_pub_nojsp.launch'
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()

    server = InteractiveMarkerServer("simple_marker")
    # rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb, server)

    rospy.Subscriber('/relaxed_ik/current_time', Float64, time_update_cb)
    
    dyn_obstacle_handles = []
    args = rospy.myargv(argv=sys.argv)
    if args[1] == "true": 
        dyn_obstacle_handles = set_collision_world(server, path_to_src, fixed_frame)

    prev_sol = starting_config
    delta_time = 0.01
    initialized = False

    rate = rospy.Rate(3000)
    while not rospy.is_shutdown():
        tf_pub.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(), 'common_world', fixed_frame)

        try: 
            param = rospy.get_param("exp_status")
            initialized = param == "go"
        except KeyError:
            initialized = False

        if initialized:
            updated = False
            for (name, waypoints) in dyn_obstacle_handles:
                if cur_time < len(waypoints) * delta_time:
                    (time, pose) = test_utils.linear_interpolate_waypoints(waypoints, int(cur_time / delta_time))
                    server.setPose(name, pose)
                    updated = True

            if updated:
                server.applyChanges()

        if len(ja_solution) == 0:
            xopt = starting_config
        else:
            xopt = ja_solution
            if not len(xopt) == len(starting_config):
                xopt = prev_sol
            else:
                prev_sol = xopt

        js = joint_state_define(xopt)
        if js == None:
            js = JointState()
            js.name = joint_ordering
            for x in xopt:
                js.position.append(x)
        js.header.stamp = rospy.Time.now()
        js_pub.publish(js)
	
        rate.sleep()

if __name__ == '__main__':
    main()
