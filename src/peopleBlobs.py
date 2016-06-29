#!/usr/bin/env python
####################################################################################
#
# This code takes the difference image, a background image, and a background mask
# and outputs a set of blob changes between the background image and the blobs
# as an array of (x, y, radius) measurements
# Note: Although it is fun to look at the video outs, they are pretty expensive.
# Note: Mask image is optional (but recommended)
#
# @version 1.0
# @author Enas
####################################################################################
import rospy
import sys
import cv2
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from sailing_stones.msg import Vector3DArray
from cv_bridge import CvBridge, CvBridgeError;


class motion_images :
  MIN_CONTOUR = 2000
  MAX_CONTOUR = 5000
  rawImage = None
  maskImage = None
  seq = 1

  def __init__(self, rawImage, diffImage, maskImage, peopleImage, peopleTracks) :
    self.image_pub = rospy.Publisher(peopleImage, Image, queue_size=25)
    self.tracks_pub = rospy.Publisher(peopleTracks, Vector3DArray, queue_size=1)

    self.bridge = CvBridge()
    self.image_rawSubscriber = rospy.Subscriber(rawImage, Image, self.rawImageCallback)
    self.image_diffSubscriber = rospy.Subscriber(diffImage, Image, self.diffImageCallback) 
    self.image_maskSubscriber = rospy.Subscriber(maskImage, Image, self.maskImageCallback) 
   

  def rawImageCallback(self, data) : # raw camera image arrives
    try :
      self.rawImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

  def maskImageCallback(self, data) : # mask camera image arrives
    try :
      inputImage = self.bridge.imgmsg_to_cv2(data, "mono8")
      ret, self.maskImage = cv2.threshold(inputImage, 128, 255, cv2.THRESH_BINARY)
    except CvBridgeError, e:
      print e


  def diffImageCallback(self, data) : # difference camera image arrives
    if self.rawImage == None :
      return
    localImage = self.rawImage.copy()
    try :
      diff = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError, e:
      print e
    
    if self.maskImage != None :
      localImage = cv2.bitwise_and(localImage, localImage, mask = self.maskImage)
      diff = cv2.bitwise_and(diff, diff, mask = self.maskImage)

    (cnts, _) = cv2.findContours(diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    points = Vector3DArray()
    for c in cnts: 
      (x, y, w, h) = cv2.boundingRect(c)
      if cv2.contourArea(c) < self.MIN_CONTOUR :
        #print "rejected contour " + str(w) + " " + str(h) + " area " + str(cv2.contourArea(c))
        continue
      if cv2.contourArea(c) > self.MAX_CONTOUR :
        #print "rejected contour " + str(w) + " " + str(h) + " area " + str(cv2.contourArea(c))
        continue
      #print "contour " + str(w) + " " + str(h) + " area " + str(cv2.contourArea(c))
      cv2.circle(localImage, (x+w/2, y+h/2), min(w/2, h/2), (0, 255, 0))
 
      vec3 = Vector3()
      vec3.x = x
      vec3.y = y
      vec3.z = min(w/2,h/2)
      points.data.append(vec3)

    try :
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(localImage, "bgr8"))
      if len(points.data) > 0 :
        points.header.stamp = rospy.Time.now()
        points.header.seq = self.seq
        self.seq = self.seq + 1
        self.tracks_pub.publish(points)
    except CvBridgeError, e:
      print e

def main(args) :

  rospy.init_node('motion_images')
  arg_defaults = {
        'rawImage': '/watcher1/image_raw',
        'maskImage' : '/watcher1/mask', 
        'diffImage': '/watcher1/changed',
        'peopleImage': '/watcher1/people',
        'peopleTracks': '/watcher1/tracks'
        }
  args = updateArgs(arg_defaults)
  motion_images(**args)
  try :
    rospy.spin()
  except KeyBoardInterrupt:
    print "Shutting down"
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
