#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;
import yaml
from PIL import Image as test
import cv

cv_image1 = None 
cv_image2 = None
cv_image3 = None
cv_image4 = None
cv_image5 = None
cv_image6 = None

allimg=[] 


class calib_points :
 
 
  def __init__(self,sink, source1, source2, source3, source4, source5, source6) :
    self.image_pub = rospy.Publisher(sink, Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub1 = rospy.Subscriber(source1, Image, self.callback1)
    self.image_sub2 = rospy.Subscriber(source2, Image, self.callback2) 
    self.image_sub3 = rospy.Subscriber(source3, Image, self.callback3)
    self.image_sub4 = rospy.Subscriber(source4, Image, self.callback4)
    self.image_sub5 = rospy.Subscriber(source5, Image, self.callback5)
    self.image_sub6 = rospy.Subscriber(source6, Image, self.callback6)

 


  def callback1(self, data1) :
    global cv_image1, cv_image2, cv_image3, cv_image4, cv_image5, cv_image6
    try :
      cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
 
    except CvBridgeError, e:
      print e


    allimg[:]=[]
   
    #cv2.cvtColor(cv_image1, cv2.COLOR_BGR2RGB, cv_image1) 
    fname = '/home/enas/cam1.yaml'
    stream = open(fname, 'r')
    ydata = yaml.load(stream)
    M=ydata['instances'][0]['R']
    M=np.array(M,dtype='float32')
    if cv_image1 != None: 
      dst = cv2.warpPerspective(cv_image1,M,(1000,1000), borderMode= cv2.BORDER_CONSTANT,borderValue=(255,0,255))
      allimg.append(dst)
    
    dsttemp2=dst.copy()

   
  
    fname2 = '/home/enas/cam2.yaml'
    stream2 = open(fname2, 'r') 
    ydata2 = yaml.load(stream2)
    M2=ydata2['instances'][0]['R']
    M2=np.array(M2,dtype='float32')
    if cv_image2 != None: 
      dst2 = cv2.warpPerspective(cv_image2,M2,(1000,1000),  borderMode= cv2.BORDER_CONSTANT,borderValue=(255,0,255))
      allimg.append(dst2)

    fname3 = '/home/enas/cam3.yaml'
    stream3 = open(fname3, 'r')
    ydata3 = yaml.load(stream3)
    M3=ydata3['instances'][0]['R']
    M3=np.array(M3,dtype='float32')
    if cv_image3 != None: 
      dst3 = cv2.warpPerspective(cv_image3,M3,(1000,1000),  borderMode= cv2.BORDER_CONSTANT,borderValue=(255,0,255))
      allimg.append(dst3)

    fname4 = '/home/enas/cam4.yaml'
    stream4 = open(fname4, 'r')
    ydata4 = yaml.load(stream4)
    M4=ydata4['instances'][0]['R']
    M4=np.array(M4,dtype='float32')
    if cv_image4 != None: 
      dst4 = cv2.warpPerspective(cv_image4,M4,(1000,1000),  borderMode= cv2.BORDER_CONSTANT,borderValue=(255,0,255))
      allimg.append(dst4)
    
    fname5 = '/home/enas/cam5.yaml'
    stream5 = open(fname5, 'r')
    ydata5 = yaml.load(stream5)
    M5=ydata5['instances'][0]['R']
    M5=np.array(M5,dtype='float32')
    if cv_image5 != None: 
      dst5 = cv2.warpPerspective(cv_image5,M5,(1000,1000), borderMode= cv2.BORDER_CONSTANT,borderValue=(255,0,255))
      allimg.append(dst5)

    fname6 = '/home/enas/cam6.yaml'
    stream6 = open(fname6, 'r')
    ydata6 = yaml.load(stream6)
    M6=ydata6['instances'][0]['R']
    M6=np.array(M6,dtype='float32')
    if cv_image6 != None: 
      dst6 = cv2.warpPerspective(cv_image6,M6,(1000,1000), borderMode= cv2.BORDER_CONSTANT,borderValue=(255,0,255))
      allimg.append(dst6)
       
    for dsti in allimg:
                 print "hello"    
                 for x in range(1000):
                   for y in range(1000): 
                       
                        A, B, C =dsttemp2[x][y]
                        A1, B1, C1 =dsti[x][y]
                        if A==255 and B==0 and C==255 :  
                         
                          dsttemp2[x][y]=dsti[x][y]
                        else:
                          if A1==255 and B1==0 and C1==255:
                            
                            dsttemp2[x][y]=dsttemp2[x][y]
                          else:
                             dsttemp2[x][y]=0.5*dsttemp2[x][y]+ 0.5*dsti[x][y]
                                    

                                    

                              


    try :
      
       
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(dsttemp2, "bgr8"))
       
       
    except CvBridgeError, e:
        print e 
   
  def callback2(self, data2) :
    global  cv_image2
    try :
      cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
    except CvBridgeError, e:
      print e

    
    #cv2.cvtColor(cv_image2, cv2.COLOR_BGR2RGB, cv_image2)

  def callback3(self, data3) :
    global  cv_image3
    try :
      cv_image3 = self.bridge.imgmsg_to_cv2(data3, "bgr8")
    except CvBridgeError, e:
      print e

      
    #cv2.cvtColor(cv_image3, cv2.COLOR_BGR2RGB, cv_image3) 
 
  def callback4(self, data4) :
    global  cv_image4 
    try :
      cv_image4 = self.bridge.imgmsg_to_cv2(data4, "bgr8")
    except CvBridgeError, e:
      print e

       
    #cv2.cvtColor(cv_image4, cv2.COLOR_BGR2RGB, cv_image4)
 
  def callback5(self, data5) :
    global  cv_image5 
    try :
      cv_image5 = self.bridge.imgmsg_to_cv2(data5, "bgr8")
    except CvBridgeError, e:
      print e

       
    #cv2.cvtColor(cv_image4, cv2.COLOR_BGR2RGB, cv_image4)

 
  def callback6(self, data6) :
    global  cv_image6 
    try :
      cv_image6 = self.bridge.imgmsg_to_cv2(data6, "bgr8")
    except CvBridgeError, e:
      print e

       
    #cv2.cvtColor(cv_image4, cv2.COLOR_BGR2RGB, cv_image4)
def main(args) :
  rospy.init_node('calib_points')

  arg_defaults = {
        'source1': '/watcher1/image_raw',
        'source2': '/watcher2/image_raw',
        'source3': '/watcher3/image_raw',
        'source4': '/watcher4/image_raw',
        'source5': '/watcher5/image_raw',
        'source6': '/watcher6/image_raw',
        'sink': '/calibratedraw/image_raw'
        }
  args = updateArgs(arg_defaults)
  calib_points(**args)
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
        print "name ", name, full_name
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
            print "We have args " + val + " value " + args[name]
    return(args)

if __name__ == '__main__':
  main(sys.argv)
