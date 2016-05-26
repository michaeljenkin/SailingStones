#!/usr/bin/env python
#####################################################################################
# Do the basic planning. This maintains a 'robotPlaces' (where the robot has been)
# This code does nothing until given a /union/moveRobot trigger
#
# @version 1.0
# @author Enas and Michael
#####################################################################################
import rospy
import sys
import cv2
import numpy as np 
import math
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;
import yaml
import cv
import gc
import heapq
from geometry_msgs.msg import Vector3
from sailing_stones.msg import Vector3DArray
from std_srvs.srv import *

class globalPlanner :
 
  def __init__(self, startloc, occupancy, nogo, been, search, waypoints, prettyMap, replanTrigger) :
    self.WIDTH = 20

    t = startloc.split(',')
    self.locn = (int(t[0]), int(t[1]))
    self.occupancy = None
    self.nogo = None
    self.been = None
    self.size = (0,0)
    self.seq = 1

    self.prettyMap_pub = rospy.Publisher(prettyMap, Image, queue_size=1)
    self.been_pub = rospy.Publisher(been, Image, queue_size=1)
    self.search_pub = rospy.Publisher(search, Image, queue_size=1)
    self.waypoints_pub = rospy.Publisher(waypoints, Vector3DArray, queue_size=1)

    self.bridge = CvBridge()

    rospy.Subscriber(occupancy, Image, self.occCallback)
    rospy.Subscriber(nogo, Image, self.nogoCallback)

    rospy.Service(replanTrigger, Trigger, self.replanCallback)

  def replanCallback(self, data) :
    if self.occupancy == None :
      return TriggerResponse(False, "no occupancy grid yet")
    if self.been == None :
      self.been = self.occupancy.copy()
      self.been.fill(1)
    self.size = self.been.shape
    self.size = (int(self.size[0]), int(self.size[1]))

    wheretogo = cv2.divide(self.occupancy, self.been)

    t = cv2.minMaxLoc(wheretogo)
    print t
    z = self.findPath(self.locn, t[3])

    if z != None :
      cv2.circle(self.been, (int(t[3][0]-self.WIDTH/2), int(t[3][0]-self.WIDTH/2)), self.WIDTH, 255, thickness=cv2.cv.CV_FILLED)
      try :
        self.been_pub.publish(self.bridge.cv2_to_imgmsg(self.been, "mono8"))
      except CvBridgeError, e:
        print e 
     
      list = Vector3DArray()
      z.reverse()
      for c in z :
        vec3 = Vector3()
        vec3.x = c[0]
        vec3.y = c[1]
        vec3.z = 0
        list.data.append(vec3)
      try :
        list.header.stamp = rospy.Time.now()
        list.header.seq = self.seq
        self.seq = self.seq + 1
        self.waypoints_pub.publish(list)
      except CvBridgeError, e:
        print e

    return TriggerResponse(True, "replanned")

# currently greedy, return c+c2 for A*
  def f(self, c, p, goalLoc) :
    c2 = math.sqrt((p[0]-goalLoc[0])**2+(p[1]-goalLoc[1])**2)
    return c2

  def adjacent(self, p) :
    res = []
    if p[0] > 0 :
      res.append((p[0]-1, p[1]))
    if p[1] > 0 :
      res.append((p[0], p[1]-1))
    if p[0] < (self.size[0] - 1) :
      res.append((p[0]+1, p[1]))
    if p[1] < (self.size[1] - 1) :
      res.append((p[0], p[1]+1))
    return res
    

  def findPath(self, startLoc, goalLoc) :
    print startLoc
    print goalLoc

    opened=[]
    closed = self.nogo.copy()
    count = 0
    heapq.heappush(opened, (self.f(0, startLoc, goalLoc), startLoc, []))
    closed[startLoc[0],startLoc[1]] = 255
    while len(opened) > 0 :
      v, cell, history = heapq.heappop(opened)
      if (cell[0] == goalLoc[0]) and (cell[1] == goalLoc[1]) :
        history.append(cell)
        return history
      else :
        adj = self.adjacent(cell)
        for p in adj :
          if closed[p[0],p[1]] == 0 :
            t = copy.copy(history)
            t.append(p)
            closed[p[0],p[1]] = 255
            count = count + 1
            if count > 10 :
              try :
                self.search_pub.publish(self.bridge.cv2_to_imgmsg(closed, "mono8"))
              except CvBridgeError, e:
                print e 
              count = 0
   
            heapq.heappush(opened, (self.f(len(t), p, goalLoc), p, t))
    return None

      

    

  def occCallback(self, data) :
    try :
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
      self.occupancy = cv_image
    except CvBridgeError, e:
      print e

  def nogoCallback(self, data) :
    try :
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
      self.nogo = cv_image
    except CvBridgeError, e:
      print e

def main(args) :
  rospy.init_node('globalPlanner')

  arg_defaults = {
     'startloc' : '539,309',
     'occupancy' : '/union/people_count',
     'nogo' : '/union/nogo',
     'been' : '/union/been',
     'search' : '/union/search',
     'waypoints' : '/union/waypoints',
     'prettyMap' : '/union/map',
     'replanTrigger' : '/union/moveRobot'
        }
  args = updateArgs(arg_defaults)
  globalPlanner(**args)
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
