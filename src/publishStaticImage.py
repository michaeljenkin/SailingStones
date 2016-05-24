#!/usr/bin/env python
########################################################################
# 
# This code takes a source image from a stationary camera and return
# an image showing changed locations in the image. It also maintains
# an estimate of the background image.
#
# @version 1.0
# @date May 9, 2016
# @author Michael Jenkin
########################################################################

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;


class publish_static_image :
 
  def __init__(self, source, output) :
    print "Publishing static image " + str(source) + " to " + str(output)
    bridge = CvBridge()
    img = cv2.imread(source, cv2.IMREAD_GRAYSCALE)
    if img == None :
      print "Unable to load image " + str(source)
    else :
      image_pub = rospy.Publisher(output, Image, queue_size=1, latch=True)
      image_pub.publish(bridge.cv2_to_imgmsg(img, "mono8"))
      print "Image " + str(source) + " published and latched"


def main(args) :
  rospy.init_node('public_static_image')

  arg_defaults = {
        'source': 'watcher1mask.jpg',
        'output': '/watcher1/mask',
        }
  args = updateArgs(arg_defaults)
  publish_static_image(**args)
  try :
    rospy.spin()
  except KeyBoardInterrupt:
    print "Shutting down (monitorChange)"
  cv2.destroyAllWindows()

def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  '''
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
    return(args)

if __name__ == '__main__':
  main(sys.argv)
