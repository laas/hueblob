#!/usr/bin/env python
import roslib; roslib.load_manifest('hueblob')

import sys

import rospy
from geometry_msgs.msg import Point
from hueblob.srv import *
import cv
from cv_bridge import CvBridge, CvBridgeError

def add_object_client(name, path):
    rospy.wait_for_service('/hueblob/add_object')
    try:
        add_object = rospy.ServiceProxy('/hueblob/add_object', AddObject)
        # load image, give anchor coordinates
        anchor = Point(0.0,0.0,0.0)
        cv_img = cv.LoadImageM(path)
        bridge = CvBridge()
        image = bridge.cv_to_imgmsg(cv_img)
        resp1  = add_object(name, anchor, image)
        return resp1.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s name path"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        name = sys.argv[1]
        path = sys.argv[2]
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s, %s"%(name, path)
    print "%s %s, %d"%(name, path, add_object_client(name, path))
