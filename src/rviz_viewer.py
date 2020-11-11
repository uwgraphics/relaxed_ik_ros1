#! /usr/bin/env python
'''
author: Danny Rakita, Haochen Shi
email: rakita@cs.wisc.edu, hshi74@wisc.edu
last update: 11/10/20

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''

import numpy
import os
import roslaunch
import rospkg
import rospy
import utils
import tf
import transformations as T
import yaml

from std_msgs.msg import ColorRGBA, Float64
from geometry_msgs.msg import Point, Pose
from interactive_markers.interactive_marker_server import *
from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from sensor_msgs.msg import JointState, PointCloud2, PointField
from timeit import default_timer as timer
from visualization_msgs.msg import *

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')
animation_folder_path = path_to_src + '/animation_files/'
geometry_folder_path = path_to_src + '/geometry_files/'
env_settings_file_path = path_to_src + '/relaxed_ik_core/config/settings.yaml'

ja_solution = ''
def ja_solution_cb(data):
    global ja_solution
    ja_solution = []
    for a in data.angles.data:
        ja_solution.append(a)

time_cur = 0.0
def time_update_cb(msg):
    global time_cur
    time_cur = msg.data
    print("The current time is {}".format(time_cur))

# def marker_feedback_cb(msg, args):
#     server = args
#     server.setPose(msg.marker_name, msg.pose)    
#     server.applyChanges()

def goal_marker_cb(msg, args):
    server = args[0]
    p = msg.ee_poses[0]
    pos_goal = numpy.array(args[1]) + numpy.array([p.position.x, p.position.y, p.position.z])
    rot_goal = T.quaternion_multiply([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z], args[2])
    pose = Pose()
    pose.position.x = pos_goal[0]
    pose.position.y = pos_goal[1]
    pose.position.z = pos_goal[2]
    pose.orientation.w = rot_goal[0]
    pose.orientation.x = rot_goal[1]
    pose.orientation.y = rot_goal[2]
    pose.orientation.z = rot_goal[3]
    server.setPose("pose_goal", pose)    
    server.applyChanges()

def print_cb(msg):
    p = msg.pose.position
    print(msg.marker_name + " is now at [" + str(p.x) + ", " + str(p.y) + ", " + str(p.z) + "]")

def make_marker(name, fixed_frame, shape, scale, ts, quat , is_dynamic, 
                points=None, color=[0.0,0.5,0.5,1.0], marker_scale=0.3):                
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = fixed_frame
    int_marker.name = name
    int_marker.pose.position.x = ts[0]
    int_marker.pose.position.y = ts[1]
    int_marker.pose.position.z = ts[2]

    int_marker.pose.orientation.x = quat[1]
    int_marker.pose.orientation.y = quat[2]
    int_marker.pose.orientation.z = quat[3]
    int_marker.pose.orientation.w = quat[0]

    int_marker.scale = marker_scale

    if shape == 'widget':
        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0
        x_axis = Point()
        x_axis.x = scale[0]
        x_axis.y = 0.0
        x_axis.z = 0.0
        y_axis = Point()
        y_axis.x = 0.0
        y_axis.y = scale[1]
        y_axis.z = 0.0
        z_axis = Point()
        z_axis.x = 0.0
        z_axis.y = 0.0
        z_axis.z = scale[2]
        points = [[origin, x_axis], [origin, z_axis], [origin, y_axis]]
        colors = [[1.0, 0.0, 0.0, 0.6], [0.0, 1.0, 0.0, 0.6], [0.0, 0.0, 1.0, 0.6]]
        for i in range(len(colors)):
            marker = Marker()
            marker.type = Marker.ARROW
            marker.scale.x = 0.01
            marker.scale.y = 0.02
            marker.scale.z = 0.03
            marker.color.r = colors[i][0]
            marker.color.g = colors[i][1]
            marker.color.b = colors[i][2]
            marker.color.a = colors[i][3]
            marker.points = points[i]

            control =  InteractiveMarkerControl()
            control.always_visible = True
            control.markers.append(marker)
            int_marker.controls.append(control)
    else:
        marker = Marker()
        marker.scale.x = scale[0] * 2
        marker.scale.y = scale[1] * 2
        marker.scale.z = scale[2] * 2
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        if shape == "cuboid":
            marker.type = Marker.CUBE
        elif shape == "sphere":
            marker.type = Marker.SPHERE
        elif shape == "point_cloud":
            marker.type = Marker.POINTS
            marker.points = points

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

        ty_control = InteractiveMarkerControl()
        ty_control.orientation.w = c
        ty_control.orientation.x = 0
        ty_control.orientation.y = 0
        ty_control.orientation.z = c
        ty_control.name = "move_y"
        ty_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(ty_control)

        tz_control = InteractiveMarkerControl()
        tz_control.orientation.w = c
        tz_control.orientation.x = 0
        tz_control.orientation.y = c
        tz_control.orientation.z = 0
        tz_control.name = "move_z"
        tz_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(tz_control)

        if shape != "sphere":
            rx_control = InteractiveMarkerControl()
            rx_control.orientation.w = c
            rx_control.orientation.x = c
            rx_control.orientation.y = 0
            rx_control.orientation.z = 0
            rx_control.name = "rotate_x"
            rx_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(rx_control)

            ry_control = InteractiveMarkerControl()
            ry_control.orientation.w = c
            ry_control.orientation.x = 0
            ry_control.orientation.y = 0
            ry_control.orientation.z = c
            ry_control.name = "rotate_y"
            ry_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(ry_control)

            rz_control = InteractiveMarkerControl()
            rz_control.orientation.w = c
            rz_control.orientation.x = 0
            rz_control.orientation.y = c
            rz_control.orientation.z = 0
            rz_control.name = "rotate_z"
            rz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(rz_control)

    return int_marker

def set_collision_world(server, fixed_frame, env_settings):
    dyn_obs_handles = []

    if 'obstacles' in env_settings:
        obstacles = env_settings['obstacles']
    else:
        raise NameError('Please define the obstacles in the environment!')

    if 'cuboids' in obstacles: 
        cuboids = obstacles['cuboids']
        if cuboids is not None:
            for c in cuboids:
                is_dynamic = c['animation'] != 'static'
                c_quat = T.quaternion_from_euler(c['rotation'][0], c['rotation'][1], c['rotation'][2])
                int_marker = make_marker(c['name'], fixed_frame, "cuboid", c['scale'], 
                                        c['translation'], c_quat, is_dynamic)
                server.insert(int_marker, print_cb)
                if is_dynamic:
                    path = animation_folder_path + c['animation']
                    relative_waypoints = utils.read_cartesian_path(path)
                    waypoints = utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                    dyn_obs_handles.append((int_marker.name, waypoints))

    if 'spheres' in obstacles:
        spheres = obstacles['spheres']
        if spheres is not None:
            for s in spheres:
                is_dynamic = s['animation'] != 'static'
                int_marker = make_marker(s['name'], fixed_frame, "sphere", [s['scale']] * 3, 
                                        s['translation'], [1.0,0.0,0.0,0.0], is_dynamic)
                server.insert(int_marker, print_cb)
                if is_dynamic:
                    path = animation_folder_path + s['animation']
                    relative_waypoints = utils.read_cartesian_path(path)
                    waypoints = utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                    dyn_obs_handles.append((int_marker.name, waypoints))

    if 'point_cloud' in obstacles: 
        point_cloud = obstacles['point_cloud']
        if point_cloud is not None:
            for pc in point_cloud:
                pc_path = geometry_folder_path + pc['file']
                pc_scale = pc['scale']
                pc_points = []
                with open(pc_path, 'r') as point_cloud_file:
                    lines = point_cloud_file.read().split('\n')
                    for line in lines:
                        pt = line.split(' ')
                        if utils.is_point(pt):
                            point = Point()
                            point.x = float(pt[0]) * pc_scale[0]
                            point.y = float(pt[1]) * pc_scale[1]
                            point.z = float(pt[2]) * pc_scale[2]
                            pc_points.append(point)
                
                is_dynamic = pc['animation'] != 'static'
                pc_quat = T.quaternion_from_euler(pc['rotation'][0], pc['rotation'][1], pc['rotation'][2])
                int_marker = make_marker(pc['name'], fixed_frame, "point_cloud", [0.01, 0.01, 0.01], pc['translation'], pc_quat, is_dynamic, points=pc_points)
                server.insert(int_marker, print_cb)
                if is_dynamic:
                    path = animation_folder_path + pc['animation']
                    relative_waypoints = utils.read_cartesian_path(path)
                    waypoints = utils.get_abs_waypoints(relative_waypoints, int_marker.pose)
                    dyn_obs_handles.append((int_marker.name, waypoints))

    server.applyChanges()

    return dyn_obs_handles

def main():
    rospy.init_node('rviz_viewer')

    env_settings_file = open(env_settings_file_path, 'r')
    env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)
    if 'loaded_robot' in env_settings:
        info_file_name = env_settings['loaded_robot']['name']
    else:
        raise NameError("Please defined the loaded robot!")
        
    info_file_path = path_to_src + '/relaxed_ik_core/config/info_files/' + info_file_name
    info_file = open(info_file_path, 'r')

    y = yaml.load(info_file, Loader=yaml.FullLoader)
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

    init_pos, init_rot = utils.get_init_pose(info_file_path)
    pose_goal_marker = make_marker('pose_goal', fixed_frame, 'widget', [0.1,0.1,0.1], init_pos, init_rot, False)
    server.insert(pose_goal_marker)

    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, goal_marker_cb, (server, init_pos, init_rot))
    rospy.Subscriber('/relaxed_ik/current_time', Float64, time_update_cb)
    
    dyn_obs_handles = []
    args = rospy.myargv(argv=sys.argv)
    if args[1] == "true": 
        dyn_obs_handles = set_collision_world(server, fixed_frame, env_settings)

    delta_time = 0.01
    prev_sol = starting_config
    initialized = False

    rate = rospy.Rate(3000)
    while not rospy.is_shutdown():
        tf_pub.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(), 'common_world', fixed_frame)

        try: 
            param = rospy.get_param("simulation_time")
            initialized = param == "go"
        except KeyError:
            initialized = False

        if initialized:
            updated = False
            for (name, waypoints) in dyn_obs_handles:
                if time_cur < len(waypoints) * delta_time:
                    (time, pose) = utils.linear_interpolate_waypoints(waypoints, int(time_cur / delta_time))
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
