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
from geometry_msgs.msg import Vector3
from sailing_stones.msg import Vector3DArray

class globalWeight :
 
 
  def __init__(self, maskSet, paramSet, trackSet, sink, nogo, dimension) :
    self.image_pub = rospy.Publisher(sink, Image, queue_size=10)
    self.nogo_pub = rospy.Publisher(nogo, Image, latch=True)

    self.masks = maskSet.split(',')
    self.yamls = paramSet.split(',')
    self.tracks = trackSet.split(',')
    self.dim =  dimension.split(',')
    self.dim = (int(self.dim[0]), int(self.dim[1]))
    self.calib = []
    self.maskImages = [None] * len(self.masks)
    self.currentBlobs = []

    print "reading in yaml files"
    for i,val in enumerate(self.yamls) :
      stream = open(val, 'r')
      ydata = yaml.load(stream)
      M = ydata['instances'][0]['R']
      M = np.array(M,dtype='float32')
      self.calib.append(M)

    self.bridge = CvBridge()

    for i,val in enumerate(self.masks) :
      self.maskSub = rospy.Subscriber(val, Image, self.maskCallback, callback_args=i)

    for i,val in enumerate(self.tracks) :
      self.trackSub = rospy.Subscriber(val, Vector3DArray, self.trackCallback, callback_args=i)

  def maskCallback(self, data, *args) :
    id = args[0]
    print "Got mask " + str(id)
    try :
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
      self.maskImages[id] = cv_image
    except CvBridgeError, e:
      print e
    allseen = True
    for x in self.maskImages :
      allseen = allseen and (x != None)

    if allseen :
      print "Publishing nogo mask"

      commonMasks = []
      for i,dsti in enumerate(self.maskImages) :
        tmp = cv2.warpPerspective(dsti,self.calib[i],self.dim, borderMode= cv2.BORDER_CONSTANT,borderValue=0)
        commonMasks.append(tmp)
      
      combined = commonMasks[0].copy()
      combined.fill(0)
      for i,img in enumerate(commonMasks) :
        combined = cv2.bitwise_or(combined, img)
      combined = cv2.bitwise_not(combined)

      try :
        self.nogo_pub.publish(self.bridge.cv2_to_imgmsg(combined, "mono8"))
       
      except CvBridgeError, e:
        print e 
   
 
  def trackCallback(self, data, *args) :
    id = args[0]
    q = (data.header.stamp.secs, id, data.data)
    heapq.heappush(self.currentBlobs, q)


    allseen = True
    for x in self.maskImages :
      allseen = allseen and (x != None)

    if allseen :
      print "Updating counts"

      counts = []
      for i,dsti in enumerate(self.maskImages) :
        t = dsti.astype(float)
        t.fill(0) 
        counts.append(t)
      

      for p in self.currentBlobs :
        cam = p[1]
        z = p[2]
        for q in z :
          t = counts[cam].copy()
          t.fill(0)
          cv2.circle(t, (int(q.x+q.z/2), int(q.y+q.z/2)), int(q.z/2), 1, thickness=cv2.cv.CV_FILLED)
          counts[cam] = cv2.add(counts[cam], t)


      common = []
      for i,dsti in enumerate(counts) :
        tmp = cv2.warpPerspective(dsti,self.calib[i],self.dim, borderMode= cv2.BORDER_CONSTANT,borderValue=0)
        common.append(tmp)

      combined = common[0].copy()
      combined.fill(0)
      for i,img in enumerate(common) :
        combined = cv2.add(combined, img)

      v = cv2.minMaxLoc(combined)
      print "minmax ", v
      v = v[1]
      if v == 0:
        v = 1.0

      combined = cv2.multiply(combined, 255/v)
      combined = np.uint8(combined)

      try :
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(combined, "mono8"))
       
      except CvBridgeError, e:
        print e 
   
      gc.collect()

def main(args) :
  rospy.init_node('globalWeight')

  arg_defaults = {
     'paramSet' : '/home/viki/catkin_ws/src/SailingStones/cfg/cam1.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam2.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam3.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam4.yaml,/home/viki/catkin_ws/src/SailingStones/cfg/cam5.yaml',
     'maskSet' : '/watcher1/mask,/watcher2/mask,/watcher3/mask,/watcher4/mask,/watcher5/mask',
     'trackSet' : '/watcher1/tracks,/watcher2/tracks,/watcher3/tracks,/watcher4/tracks,/watcher5/tracks',
     'sink' : '/union/people_count',
     'nogo' : '/union/nogo',
     'dimension' : '1000,1000'
        }
  args = updateArgs(arg_defaults)
  globalWeight(**args)
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
