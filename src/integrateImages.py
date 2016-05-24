#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;


class integrate_images :
  history = None
  nframes = 100
  frame = 1
 
  def __init__(self, source, sink) :
    self.image_pub = rospy.Publisher(sink, Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(source, Image, self.callback)


  def callback(self, data) :
    try :
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape

    frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    frame = frame.astype('float')
#    frame = cv2.GaussianBlur(frame, (21, 21), 0)
    if self.history == None :
      self.history = frame
      self.frame = 1
    else :
      dst = cv2.add(self.history, frame)
      self.history = dst
      self.frame = self.frame + 1

   # print "processing frame %d of %d" % (self.frame, self.nframes)
    if self.frame == self.nframes :
      try :
        dst = self.history
        min, max, loc1, loc2 = cv2.minMaxLoc(dst)
       # print min, max
        dst = np.array(dst * 255.0 / max, dtype=np.uint8)
        dst= cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))
        self.history = None
       # print "frame published"
      except CvBridgeError, e:
        print e

def main(args) :

  rospy.init_node('integrate_images')
  arg_defaults = {
        'source': '/averaged/changed',
        'sink': '/averaged/averaged'
        }
  args = updateArgs(arg_defaults)
  integrate_images(**args)
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
