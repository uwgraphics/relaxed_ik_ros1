#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/8/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''

import rospy
import yaml
import roslaunch
import tf
import os
from sensor_msgs.msg import JointState
from relaxed_ik_ros1.msg import JointAngles

import os.path
import transformations as T
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
import numpy

ja_solution = ''
def ja_solution_cb(data):
    global ja_solution
    ja_solution = []
    for a in data.angles.data:
        ja_solution.append(a)

def processFeedback(feedback):
    p = feedback.pose.position
    print (feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

def makeMarker(name, fixed_frame, shape, ts, rots, scale, is_dynamic):                
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
    if shape == "cuboid":
        marker.type = Marker.CUBE
    else:
        marker.type = Marker.SPHERE

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

        if shape == "cuboid":
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
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(rz_control)

            ry_control = InteractiveMarkerControl()
            ry_control.orientation.w = c
            ry_control.orientation.x = 0
            ry_control.orientation.y = 0
            ry_control.orientation.z = c
            ry_control.name = "rotate_y"
            ry_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(ry_control)

    server.insert(int_marker, processFeedback)

    return int_marker

def is_point(pt):
    if len(pt) < 3:
        return False
    for e in pt:
        try:
            float(e)
        except ValueError:
            return False
    return True

if __name__ == '__main__':
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

    pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2, queue_size=10)
    points_msgs = []

    env_collision_on = True

    if env_collision_on: 
        env_collision_file_path = path_to_src + '/env_collision_files/env_collision.yaml'
        
        if os.path.exists(env_collision_file_path):
            env_collision_file = open(env_collision_file_path, 'r')
            env_collision = yaml.load(env_collision_file)
            
            server = InteractiveMarkerServer("simple_marker")

            if 'boxes' in env_collision: 
                planes = env_collision['boxes']
                if planes is not None:
                    for i, p in enumerate(planes):
                        makeMarker(p['name'], fixed_frame, "cuboid", p['translation'], p['rotation'], p['parameters'], p['is_dynamic'])

            if 'spheres' in env_collision:
                spheres = env_collision['spheres']
                if spheres is not None:
                    for i, s in enumerate(spheres):
                        radius = s['parameters']
                        int_marker = makeMarker(s['name'], fixed_frame, "sphere", s['translation'], [0, 0, 0], [radius, radius, radius], s['is_dynamic'])
                        # print(int_marker)
            
            server.applyChanges()

            if 'point_cloud' in env_collision: 
                point_cloud = env_collision['point_cloud']
                if point_cloud is not None:
                    for pc in point_cloud:
                        scale = pc['scale']
                        ts = pc['translation']
                        rots = pc['rotation']

                        header = Header()
                        header.stamp = rospy.Time.now()
                        header.frame_id = fixed_frame
                        fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1), 
                            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]

                        pcd_path = path_to_src + '/env_collision_files/' + pc['file']
                        points = []
                        with open(pcd_path, 'r') as point_cloud_file:
                            lines = point_cloud_file.read().split('\n')
                            for line in lines:
                                pt = line.split(' ')
                                if is_point(pt):
                                    pt_list = [float(pt[0]), float(pt[1]), float(pt[2]), 0.0]
                                    rot_mat = T.euler_matrix(rots[0], rots[1], rots[2])
                                    pt_rot = numpy.matmul(rot_mat, pt_list).tolist()
                                    pt_new = [scale[0]*pt_rot[0]+ts[0], scale[1]*pt_rot[1]+ts[1], scale[2]*pt_rot[2]+ts[2]]
                                    points.append(pt_new)

                        points_msgs.append(pcl2.create_cloud(header, fields, points))
                        
    prev_state = []

    rate = rospy.Rate(200.0)
    prev_sol = starting_config
    while not rospy.is_shutdown():
        tf_pub.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(), 'common_world', fixed_frame)
        
        if len(points_msgs) > 0:
            for msg in points_msgs:
                msg.header.stamp = rospy.Time.now()
                pcl_pub.publish(msg)

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
