#! /usr/bin/env python

from geometry_msgs.msg import Pose

def readCartesianPath(filename):
    file = open(filename, 'r')
    lines = file.readlines()
    waypoints = []
    for line in lines:
        line = line.strip()
        info = line.split(';')
        position = info[1].split(',')
        orientation = info[2].split(',')

        wpose = Pose()
        wpose.position.x = float(position[0])
        wpose.position.y = float(position[1])
        wpose.position.z = float(position[2])
        wpose.orientation.w = float(orientation[0])
        wpose.orientation.x = float(orientation[1])
        wpose.orientation.y = float(orientation[2])
        wpose.orientation.z = float(orientation[3])
        waypoints.append(wpose)

    return waypoints