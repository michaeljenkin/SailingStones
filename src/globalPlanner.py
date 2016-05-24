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
 
  def __init__(self, startloc, occupancy, nogo, been, waypoints, prettyMap, peopleTrigger, replanTrigger) :
    self.ALPHA = 0.1

    t = startloc.split(',')
    self.locn = (t[0], t[1])
    self.occupancy = None
    self.nogo = None
    self.been = None
    self.size = (0,0)

    self.prettyMap_pub = rospy.Publisher(prettyMap, Image, queue_size=1)
    self.been_pub = rospy.Publisher(been, Image, queue_size=1)

    self.bridge = CvBridge()

    rospy.Subscriber(occupancy, Image, self.occCallback)
    rospy.Subscriber(nogo, Image, self.nogoCallback)

    rospy.Service(replanTrigger, Trigger, self.replanCallback)

  def replanCallback(self, data) :
    if self.been == None :
      self.been = self.occupancy.copy()
      self.been.fill(0)
    self.size = self.been.shape()[0:1]
    print self.size

    wheretogo = cv2.addWeighted(self.been, self.ALPHA, self.occupancy, 1-self.ALPHA, 0)
    t = cv2.minMaxLoc(wheretogo)
    z = self.findPath(self.locn, t)
    print z

    return TriggerResponse(True, "replanned")

  def f(self, p, startLoc, goalLoc) :
    c1 = math.sqrt((p[0]-startLoc[0])**2+(p[1]-startLoc[1])**2)
    c2 = math.sqrt((p[0]-goalLoc[0])**2+(p[1]-goalLoc[1]))*2)
    return c1 + c2

  def adj(self, p) :
    res = []
    if p[0] > 0 :
      res.append((p[0]-1, p[1]))
    if p[1] > 0 :
      res.append((p[0], p[1]-1))
    if p[0] < (self.size[0] - 1)
      res.append((p[0]+1, p[1]))
    if p[1] < (self.size[1] - 1)
      res.append((p[0], p[1]+1))
    

  def findPath(self, startLoc, goalLoc) :
    print startLoc
    print goalLoc

    opened=[]
    closed = []
    heapq.push(opened, (self.f(startLoc, startLoc, goalLoc), startLoc)
    while len(opened) :
      v, cell = heapq.heappop(opened)
      closed.add(cell)
      if cell is goalLoc :
        print "found path"
      else :
        adj = self.adjacent(cell)
        for p in adj :
          if (self.nogo[p[0],p[1]] == 255) && p not in closed :
            heapq.push(opened, (self.f(p, startLoc, goalLoc), p)

      

    

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
     'startloc' : '100,100',
     'occupancy' : '/union/people_count',
     'nogo' : '/union/nogo',
     'been' : '/union/been',
     'waypoints' : '/union/waypoints',
     'prettyMap' : '/union/map',
     'peopleTrigger' : '/union/reset',
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
