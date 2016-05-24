#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;
# import the necessary packages
import datetime
import time
import math

cv_image1 = None 
cv_image2 = None


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
  

  def distance(self, p0, p1): 
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


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
      

  def callback2(self, data2) :
    global cv_image1, cv_image2
    try :
      cv_image2 = self.bridge.imgmsg_to_cv2(data2, "mono8")
    except CvBridgeError, e:
      print e
    
    frame = cv_image1  
    (rows,cols,channels) = cv_image2.shape
    #thresh = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2GRAY)
    (cnts, _) = cv2.findContours(cv_image2.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

# loop over the contours
    for c in cnts: 
            if self.i==0:
		# if the contour is too small, ignore it
		if cv2.contourArea(c) < 2000:
			continue

		# compute the bounding box for the contour, draw it on the frame,
		# and update the text

		(x, y, w, h) = cv2.boundingRect(c)
                if not (x==0 or y==0) :  
      
                  self.refPt.append((x,y,w,h))
                  roi = frame[y:y+h, x:x+w]
                  histroi = cv2.calcHist(roi,[0,1,2],None,[8,8,8],[0, 256, 0, 256, 0, 256])
                  histroi = cv2.normalize(histroi).flatten()
                  self.windowhist.append(histroi)
                  
                  self.path.append([])
                  self.path[0].append((x,y,w,h))
                  
                  #print "first", path
                  #print "first", windowhist
                  #print "first", refPt        
                  self.i=self.i+1

            elif self.i>0:
        
               # if the contour is too small, ignore it
		if cv2.contourArea(c) < 2000:
			continue

		# compute the bounding box for the contour, draw it on the frame,
		# and update the text

		(x, y, w, h) = cv2.boundingRect(c)
                #self.firstframe.itemset(( y+h/2,x+w/2,2),255)
                cv2.circle(self.firstframe, (x+w/2, y+h/2),5, (0, 255, 0), -1)
                if not (x==0 or y==0) :  
		  cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0),2)
                  roi2 = frame[y:y+h, x:x+w]
                  histroi2 = cv2.calcHist(roi2,[0,1,2],None,[8,8,8],[0, 256, 0, 256, 0, 256])
                  histroi2 = cv2.normalize(histroi2).flatten()
                   
                  s=0
                  for n in self.windowhist:
                     (a,b,c,d)=self.refPt[s]
                     self.comparehist.append(cv2.compareHist(histroi2,n,cv2.cv.CV_COMP_CHISQR))#*distance((x,y),(a,b)))
                     self.dist.append(self.distance((x,y),(a,b)))  
                     s=s+1
                 
                  print "cordinate", x,y 
                    
                  self.index=self.dist.index(min(self.dist))
                  print "index", self.index
                       
                  (a,b,c,d)=self.refPt[self.index]
                  print self.refPt[self.index] 
                  
                  print self.dist[self.index]  
                  print self.comparehist[self.index]         
                  if self.comparehist[self.index] < 2.0 and self.dist[self.index]< 100: 
                    self.path[self.index].append((x,y,w,h))
                    if len(self.path[self.index])>2:    
                   
                           (e,f,g,h)=self.path[self.index][len(self.path[self.index])-1]
                           (k,l,m,p)=self.path[self.index][len(self.path[self.index])-2]               
                           cv2.line(self.firstframe,(e+g/2,f+h/2),(k+m/2,l+p/2),(255,0,0),5)
                           print "line written --------------------------------"
                           
                    self.refPt[self.index]=(x,y,w,h)
                    print self.refPt[self.index]                  
                    self.windowhist[self.index]=histroi2
                  else :
                    self.refPt.append((x,y,w,h))
                    self.windowhist.append(histroi2)
                    self.path.append([])
                    self.path[len(self.path)-1].append((x,y,w,h))                        
                    #print "2nd", refPt
                    #print  "2nd", path
                  self.comparehist[:]=[]
                  self.dist[:]=[] 
                  #print self.comparehist
                  #print self.dist     
                  
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
