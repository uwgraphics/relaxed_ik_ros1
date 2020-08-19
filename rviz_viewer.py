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

ja_solution = ''
def ja_solution_cb(data):
    global ja_solution
    ja_solution = []
    for a in data.angles.data:
        ja_solution.append(a)

def processFeedback(feedback):
    p = feedback.pose.position
    print (feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

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

    if 'env_collision_file_name' in y: 
        env_collision_file_path = path_to_src + '/relaxed_ik_core/config/env_collision_files/' + y['env_collision_file_name']
        
        if os.path.exists(env_collision_file_path):
            env_collision_file = open(env_collision_file_path, 'r')
            env_collision = yaml.load(env_collision_file)

            planes = env_collision['boxes']
            spheres = env_collision['spheres']

            # create an interactive marker server on the topic namespace simple_marker
            server = InteractiveMarkerServer("simple_marker")

            # create an interactive marker for our server
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = fixed_frame
            int_marker.name = "my_marker"
            
            if planes is not None and len(planes) > 0: 
                for p in planes:
                    ts = p['translation']
                    rots = p['rotation']
                    dim = p['parameters']
                    plane_marker = Marker()
                    plane_marker.type = Marker.CUBE
                    plane_marker.pose.position.x = ts[0]
                    plane_marker.pose.position.y = ts[1]
                    plane_marker.pose.position.z = ts[2]

                    quat = T.quaternion_from_euler(rots[0], rots[1], rots[2])
                    plane_marker.pose.orientation.x = quat[1]
                    plane_marker.pose.orientation.y = quat[2]
                    plane_marker.pose.orientation.z = quat[3]
                    plane_marker.pose.orientation.w = quat[0]

                    plane_marker.scale.x = dim[0] * 2
                    plane_marker.scale.y = dim[1] * 2
                    plane_marker.scale.z = dim[2] * 2
                    plane_marker.color.r = 0.0
                    plane_marker.color.g = 0.5
                    plane_marker.color.b = 0.5
                    plane_marker.color.a = 1.0

                    plane_control = InteractiveMarkerControl()
                    plane_control.always_visible = True
                    plane_control.markers.append(plane_marker)

                    int_marker.controls.append(plane_control)
                
                server.insert(int_marker, processFeedback)
                server.applyChanges()

            if spheres is not None and len(spheres) > 0: 
                for s in spheres:
                    ts = s['translation']
                    radius = s['parameters']
                    sphere_marker = Marker()
                    sphere_marker.type = Marker.SPHERE
                    sphere_marker.pose.position.x = ts[0]
                    sphere_marker.pose.position.y = ts[1]
                    sphere_marker.pose.position.z = ts[2]
                    sphere_marker.pose.orientation.x = 0.0
                    sphere_marker.pose.orientation.y = 0.0
                    sphere_marker.pose.orientation.z = 0.0
                    sphere_marker.pose.orientation.w = 1.0
                    sphere_marker.scale.x = radius * 2
                    sphere_marker.scale.y = radius * 2
                    sphere_marker.scale.z = radius * 2
                    sphere_marker.color.r = 0.0
                    sphere_marker.color.g = 0.5
                    sphere_marker.color.b = 0.5
                    sphere_marker.color.a = 1.0

                    sphere_control = InteractiveMarkerControl()
                    sphere_control.always_visible = True
                    sphere_control.markers.append(sphere_marker)

                    int_marker.controls.append(sphere_control)

                server.insert(int_marker, processFeedback)
                server.applyChanges()

    prev_state = []

    rate = rospy.Rate(200.0)
    prev_sol = starting_config
    while not rospy.is_shutdown():
        tf_pub.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(), 'common_world', fixed_frame)
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
        now = rospy.Time.now()
        js.header.stamp.secs = now.secs
        js.header.stamp.nsecs = now.nsecs
        js_pub.publish(js)
	
    rate.sleep()
