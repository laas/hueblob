#!/usr/bin/env python
import roslib; roslib.load_manifest('hueblob')
import rospy
import cv_bridge
import cv
import sys

from hueblob.msg import Blobs
from sensor_msgs.msg import Image

windowName = "2d tracking result"
image = None
font = None

def callbackImage(msg):
    global image
    br = cv_bridge.CvBridge()
    image = br.imgmsg_to_cv(msg)

def callbackBlobs(blobs):
    global image
    if not image:
        return
    imageWithBlob = cv.CreateMat(image.rows, image.cols, cv.CV_8UC3)
    cv.Copy(image, imageWithBlob)
    txt = "Tracking objects: "
    for b in blobs.blobs:
        bottom_right = (int(b.boundingbox_2d[0]), int(b.boundingbox_2d[1]))
        top_left = (int(b.boundingbox_2d[0] + b.boundingbox_2d[2]),
                    int(b.boundingbox_2d[1] + b.boundingbox_2d[3]))
        cv.Rectangle(imageWithBlob,
                     top_left, bottom_right, cv.Scalar(255, 0, 0))
        txt += b.name + " "

    txt_coord = (int(0.01 * imageWithBlob.cols), int(0.85 * imageWithBlob.rows))
    cv.PutText(imageWithBlob, txt, txt_coord, font, cv.Scalar(0,0,0))

    cv.NamedWindow(windowName)
    cv.ShowImage(windowName, imageWithBlob)

class DisplayTrackingNode:
    def __init__(self, stereo, image, blob):
        global font
        rospy.Subscriber(blob, Blobs, callbackBlobs)
        rospy.Subscriber(stereo + "/left/" + image,
                         Image, callbackImage)

        print "Subscribing to:"
        print "\t* " + blob
        print "\t* " + stereo + "/left/" + image

        cv.StartWindowThread()
        cv.NamedWindow(windowName)
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.20, 1,
                           thickness = 2)

def main():
    from optparse import OptionParser
    rospy.init_node('hueblob_tracking_client', anonymous=True)
    parser = OptionParser()
    parser.add_option("-s", "--stereo",
                      dest="stereo", default="",
                      help="stereo prefix")
    parser.add_option("-i", "--image",
                      dest="image",
                      default="image_rect_color",
                      help="image topic name")
    parser.add_option("-b", "--blob",
                      dest="blob",
                      default="/hueblob/blobs",
                      help="blobs topic")
    options, args = parser.parse_args()

    stereo = options.stereo
    image = options.image
    blob = options.blob

    if not stereo or not image or not blob:
        print("Please set all the required information, " +
              "see --help for more information")
        sys.exit(1)

    node = DisplayTrackingNode(stereo, image, blob)
    rospy.spin()

if __name__ == '__main__':
    main()
