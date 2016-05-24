#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;
# import the necessary packages
from datetime import datetime, timedelta
import time
import math
import matplotlib.pylab as plt

cv_image1 = None 
cv_image2 = None
a= datetime.now()
minu= timedelta(minutes=10)
robotpath= np.zeros((64,48))

class motion_images :
  history = None
  nframes = 100
  frame = 1
  refPt=[]
  windowhist=[]
  comparehist=[]
  dist=[]
  path=[]
  firstframe= None
  i=0
  index=0
  


  def __init__(self, source1,source2,sink) :
    self.image_pub = rospy.Publisher(sink, Image, queue_size=25)

    self.bridge = CvBridge()
    self.image_sub1 = rospy.Subscriber(source1, Image, self.callback1)
    self.image_sub2 = rospy.Subscriber(source2, Image, self.callback2) 
   

  def callback1(self, data1) :
    global cv_image1, cv_image2
    try :
      cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image1.shape    
    if self.firstframe == None: 
       self.firstframe = cv_image1.copy()
       cv2.rectangle(self.firstframe, (0, 0), (640, 480), (255,0,0), thickness=cv2.cv.CV_FILLED)
       a = datetime.now() 
      

  def callback2(self, data2) :
    global cv_image1, cv_image2, a
    try :
      cv_image2 = self.bridge.imgmsg_to_cv2(data2, "mono8")
    except CvBridgeError, e:
      print e
    
    frame = cv_image1  
    (rows,cols,channels) = cv_image2.shape
    #thresh = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2GRAY)
    b = datetime.now()
    c = b-a 
    if c>minu:
        print minu
        print c
        a = datetime.now()
        print a
        #self.firstframe = cv_image1.copy()

    (cnts, _) = cv2.findContours(cv_image2.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

# loop over the contours
    for c in cnts: 
         
        
               # if the contour is too small, ignore it
		if cv2.contourArea(c) < 2000:
			continue

		# compute the bounding box for the contour, draw it on the frame,
		# and update the text

		(x, y, w, h) = cv2.boundingRect(c)

               
		#cv2.rectangle(self.firstframe, (x, y), (x + w, y + h), (0, 255, 0),2)
                M=(x+w/2)/10
                N=(y+h/2)/10
                #print "mn", M, N
                robotpath[M,N]=robotpath[M,N]+1
                cv2.circle(self.firstframe, (x+w/2, y+h/2),10, (0, 255-robotpath[M,N], 0), -1) 


               
    try :
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.firstframe, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args) :

  rospy.init_node('motion_images')
  arg_defaults = {
        'source1': '/watcher1/image_raw',
        'source2': 'changed1/image_raw',
        'sink': 'path/image_raw'
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
    print "processing args"
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
