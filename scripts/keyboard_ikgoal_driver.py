#!/usr/bin/python3

import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
import transformations as T

rospy.init_node('keyboard_ikgoal_driver')

ee_vel_goals_pub = rospy.Publisher('relaxed_ik/ee_vel_goals', EEVelGoals, queue_size=5)

pos_stride = 0.005
rot_stride = 0.030

seq = 1
rate = rospy.Rate(1000)
while not rospy.is_shutdown():

    linear = [0,0,0]
    angular = [0,0,0]

    key = readchar.readkey()
    if key == 'w':
        linear[0] += pos_stride
    elif key == 'x':
        linear[0] -= pos_stride
    elif key == 'a':
        linear[1] += pos_stride
    elif key == 'd':
        linear[1] -= pos_stride
    elif key == 'q':
        linear[2] += pos_stride
    elif key == 'z':
        linear[2] -= pos_stride
    elif key == '1':
        angular[0] += rot_stride
    elif key == '2':
        angular[0] -= rot_stride
    elif key == '3':
        angular[1] += rot_stride
    elif key == '4':
        angular[1] -= rot_stride
    elif key == '5':
        angular[2] += rot_stride
    elif key == '6':
        angular[2] -= rot_stride

    elif key == 'q':
        q = Bool()
        q.data = True
    elif key == 'c':
        rospy.signal_shutdown()

    print("Linear : {}, Angular: {}".format(linear, angular))

    twist = Twist()
    twist.linear.x = linear[0]
    twist.linear.y = linear[1]
    twist.linear.z = linear[2]

    twist.angular.x = angular[0]
    twist.angular.y = angular[1]
    twist.angular.z = angular[2]


    tolerance = Twist()
    tolerance.linear.x = 0.0
    tolerance.linear.y = 0.0
    tolerance.linear.z = 0.0
    tolerance.angular.x = 0.0
    tolerance.angular.y = 0.0
    tolerance.angular.z = 0.0

    msg = EEVelGoals()
    msg.ee_vels.append(twist)
    msg.tolerances.append(tolerance)

    seq += 1
    ee_vel_goals_pub.publish(msg)
