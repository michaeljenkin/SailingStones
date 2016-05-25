#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;

template = None
tempcircle = []

smallcircle = []
Bigcircle = []


class find_robot:
    firstframe = None

    def __init__(self, source, sink):

        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber(source, Image, self.callback)
        self.image_pub = rospy.Publisher(sink, Image, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        (rows, cols, channels) = cv_image.shape

        # load the image, clone it for output, and then convert it to grayscale

        output = cv_image.copy()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # gray = cv2.medianBlur(gray, 5)
        template = cv2.imread("/home/enas/template.jpg")
        if template is None:
            print "None"
        template = cv2.cvtColor(template, cv2.COLOR_RGB2GRAY)

        result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        top_left = max_loc
        top_left = (top_left[0] - 10, top_left[1] - 10)
        h, w = template.shape
        bottom_right = (top_left[0] + w + 20, top_left[1] + h + 20)
        if 60000000 < max_val < 80000000:
            cv2.rectangle(output, top_left, bottom_right, (0, 0, 255), 3)

        # detect circles in the image
        circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 2.2, 25.0, minRadius=20, maxRadius=66)
        # print circles
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")

            # loop over the (x, y) coordinates and radius of the circle
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                if top_left[0] < x < bottom_right[0] and top_left[1] < y < bottom_right[
                    1] and 55000000 < max_val < 80000000:
                    tempcircle.append((x, y, r))

                # show the output image
                # output=np.hstack([cv_image, output]))
                # print tempcircle, max_val
            tempcircle[:] = []

        try:

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))


        except CvBridgeError, e:
            print e


def main(args):
    rospy.init_node('find_robot')
    arg_defaults = {
        'source': '/watcher/image_raw',
        'sink': '/findrobot/image_raw',
    }
    args = updateArgs(arg_defaults)
    find_robot(**args)
    try:
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
    return (args)


if __name__ == '__main__':
    main(sys.argv)
