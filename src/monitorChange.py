#!/usr/bin/env python
########################################################################
# 
# This code takes a source image from a stationary camera and return
# an image showing changed locations in the image. It also maintains
# an estimate of the background image.
#
# @version 2.0
# @date May 9, 2016
# @author Enas
########################################################################

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;


class monitor_change :
  ALPHA = 0.9 # parameter for background filter
  SIGMA = 21 # Gaussian blur sigma
  THRESHOLD = 25 # threshold for image difference
  background = None
 
  def __init__(self, source, sink, background) :
    print "Monitoring change from " + str(source) + " to " + str(sink)
    self.image_pub = rospy.Publisher(sink, Image, queue_size=25)
    self.back_pub = rospy.Publisher(background, Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(source, Image, self.callback)


  def callback(self, data) :
    try :
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape

    frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (self.SIGMA, self.SIGMA), 0)
    if self.background == None :
      self.background = frame
    else:
      cv2.addWeighted(self.background,self.ALPHA,frame,1-self.ALPHA,0.0,self.background)


    frameDelta = cv2.absdiff(self.background, frame)
    thresh = cv2.threshold(frameDelta, self.THRESHOLD, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    
    try :
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "mono8"))
      self.back_pub.publish(self.bridge.cv2_to_imgmsg(self.background, "mono8"))
    except CvBridgeError, e:
      print e

def main(args) :
  rospy.init_node('monitor_change')

  arg_defaults = {
        'source': '/watcher1/image_raw',
        'sink': '/watcher1/changed',
        'background' : '/watcher1/background'
        }
  args = updateArgs(arg_defaults)
  monitor_change(**args)
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
