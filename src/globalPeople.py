#!/usr/bin/env python
#####################################################################################
# Merge a collection of images together. This requires the image set
# as well as a matching set of calibration yaml files to define the 
# homographies between the cameras and a common world
#
# @version 1.0
# @author Enas and Michael
#####################################################################################
import rospy
import sys
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;
import yaml
import cv
import gc
import heapq
from threading import Lock
from geometry_msgs.msg import Vector3
from sailing_stones.msg import Vector3DArray

class globalImage :
 
 
  def __init__(self, cameraSet, maskSet, paramSet, trackSet, sink, dimension) :
    self.image_pub = rospy.Publisher(sink, Image, queue_size=10)

    self.cameras = cameraSet.split(',')
    self.masks = maskSet.split(',')
    self.yamls = paramSet.split(',')
    self.tracks = trackSet.split(',')
    self.dim =  dimension.split(',')
    self.dim = (int(self.dim[0]), int(self.dim[1]))
    self.calib = []
    self.images = [None] * len(self.cameras)
    self.maskImages = [None] * len(self.masks)
    self.currentBlobs = []
    self.lock = Lock()

    print "reading in yaml files"
    for i,val in enumerate(self.yamls) :
      stream = open(val, 'r')
      ydata = yaml.load(stream)
      M = ydata['instances'][0]['R']
      M = np.array(M,dtype='float32')
      self.calib.append(M)

    self.bridge = CvBridge()
    for i,val in enumerate(self.cameras) :
      self.camSub = rospy.Subscriber(val, Image, self.camCallback, callback_args=i)

    for i,val in enumerate(self.masks) :
      self.maskSub = rospy.Subscriber(val, Image, self.maskCallback, callback_args=i)

    for i,val in enumerate(self.tracks) :
      self.trackSub = rospy.Subscriber(val, Vector3DArray, self.trackCallback, callback_args=i)

  def trackCallback(self, data, *args) :
    self.lock.acquire()
    id = args[0]
    q = (data.header.stamp.secs, id, data.data)
    heapq.heappush(self.currentBlobs, q)
    self.lock.release()

  def maskCallback(self, data, *args) :
    self.lock.acquire()
    id = args[0]
    try :
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
      self.maskImages[id] = cv_image
    except CvBridgeError, e:
      print e
    self.lock.release()
 
  def camCallback(self, data, *args) :
    self.lock.acquire()
    id = args[0]

    try :
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.images[id] = cv_image
    except CvBridgeError, e:
      print e

    allseen = True
    for x in self.images :
      allseen = allseen and (x != None)
    for x in self.maskImages :
      allseen = allseen and (x != None)

    if allseen :
      print "merging all images into a common reference frame"

      for p in self.currentBlobs :
        cam = p[1]
        z = p[2]
        for q in z :
          cv2.circle(self.images[cam], (int(q.x+q.z/2), int(q.y+q.z/2)), int(q.z/2), (0, 255, 0), thickness=cv2.cv.CV_FILLED)

      common = []
      for i,dsti in enumerate(self.images) :
        tmp = cv2.warpPerspective(dsti,self.calib[i],self.dim, borderMode= cv2.BORDER_CONSTANT,borderValue=(255,0,255))
        common.append(tmp)

      commonMasks = []
      for i,dsti in enumerate(self.maskImages) :
        tmp = cv2.warpPerspective(dsti,self.calib[i],self.dim, borderMode= cv2.BORDER_CONSTANT,borderValue=(0,0,0))
        commonMasks.append(tmp)
      
      combined = common[0].copy()
      cv2.rectangle(combined, (0,0), (self.dim[0]-1,self.dim[1]-1), (255,0,255), thickness=cv2.cv.CV_FILLED)
      for i,img in enumerate(common) :
        notmask = cv2.bitwise_not(commonMasks[i])
        combined = cv2.bitwise_and(combined, combined, mask=notmask)
        img = cv2.bitwise_and(img, img, mask= commonMasks[i])
        combined = cv2.add(combined, img)

      try :
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(combined, "bgr8"))
       
      except CvBridgeError, e:
        print e 
   
      self.images = [None] * len(self.cameras)
      gc.collect()
    self.lock.release()

def main(args) :
  rospy.init_node('globalImage')

  arg_defaults = {
     'cameraSet' : '/watcher1/image_raw,/watcher2/image_raw,/watcher3/image_raw,/watcher4/image_raw,/watcher5/image_raw',
     'paramSet' : '/home/viki/catkin_ws/src/SailingStones/cfg/cam1.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam2.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam3.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam4.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam5.yaml',
     'maskSet' : '/watcher1/mask,/watcher2/mask,/watcher3/mask,/watcher4/mask,/watcher5/mask',
     'trackSet' : '/watcher1/tracks,/watcher2/tracks,/watcher3/tracks,/watcher4/tracks,/watcher5/tracks',
     'sink' : '/union/people_raw',
     'dimension' : '1000,1000'
        }
  args = updateArgs(arg_defaults)
  globalImage(**args)
  try :
    rospy.spin()
  except KeyBoardInterrupt:
    print "Shutting down"

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
      print "We have args " + val + " value " + args[name]
  return(args)

if __name__ == '__main__':
  main(sys.argv)
