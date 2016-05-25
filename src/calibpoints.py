#!/usr/bin/env python
import rospy, rospkg
import sys
import cv2
from os.path import join, isfile
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;
import yaml

class calib_points:
    def __init__(self, calib_topic, image_topics):
        self.image_pub = rospy.Publisher(calib_topic, Image, queue_size=10)
        self.bridge = CvBridge()

        # wait for slow topics
        rospy.sleep(rospy.Duration(0.5))

        published_topics = rospy.get_published_topics()

        # only interested in published topics
        self.img_topics = [topic for topic in image_topics if topic in published_topics]
        self.img_calib = [None for _ in self.image_topics]
        self.images = [None for _ in self.image_topics]

        for img_topic in self.image_topics:
            rospy.Subscriber(img_topic, Image, self.image_callback, img_topic)

        self.load_calibration_data()
        pass

    def load_calibration_data(self):
        package = rospkg.RosPack().get_path('sailing_stones')

        for idx in range(len(self.img_topics)):
            fname = join(package, 'cfg/cam%i.yaml' % idx)

            if not isfile(fname):
                rospy.logerr("Configuration file %f doesn't exist" % fname)
                continue

            yaml_data = yaml.load(open(fname, 'r'))
            self.img_calib = np.array(yaml_data['instances'][0]['R'], dtype='float32')
        pass

    def image_callback(self, msg, topic):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            return

        index = self.image_topics.index(topic)
        image = cv2.warpPerspective(cv_image, self.img_calib, (1000, 1000), borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=(255, 0, 255))
        self.images[index] = image

        if index != 0:
            return

        output_image = image.copy()

        for img in self.images:
            for x in range(1000):
                for y in range(1000):
                    if output_image[x][y] == (255, 0, 255):
                        output_image[x][y] = img[x][y]
                    elif img[x][y][0] == 0 and img[x][y][2] == 0 and img[x][y][1] > output_image[x][y][1]:
                        output_image[x][y] = img[x][y]
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "bgr8"))
        except CvBridgeError, e:
            print e
        pass

def main(args):
    rospy.init_node('calib_points')

    arg_defaults = {
        'image_topics' : ['/path1/image_raw', '/path2/image_raw', '/path3/image_raw', '/path4/image_raw', '/path5/image_raw', '/path6/image_raw'],
        'calib_topic': '/calibrated/image_raw'
    }

    args = updateArgs(arg_defaults)
    calib_points(**args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException, e:
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
    return (args)


if __name__ == '__main__':
    main(sys.argv)
