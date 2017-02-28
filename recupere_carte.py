#! /usr/bin/env python

import sys
import time
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/home/joris/new_ws/devel/lib/python2.7/dist-packages")

import rospy

from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid

rospy.init_node('recupere_carte')

def param_map(msg):
    print msg.map_load_time
    print msg.resolution
    print msg.width
    print msg.height
    print msg.origin


def affiche_map(msg):
    print msg.info
    print msg.data

while not rospy.is_shutdown():
    #sub = rospy.Subscriber('map_metadata', MapMetaData, param_map)
    time.sleep(1)
    sub2 = rospy.Subscriber('map', OccupancyGrid, affiche_map)

    #rospy.spin()