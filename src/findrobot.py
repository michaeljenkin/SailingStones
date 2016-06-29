#!/usr/bin/env python
import rospy, rospkg
import sys
import cv2
import copy

from sklearn.naive_bayes import GaussianNB
from sklearn.neighbors.nearest_centroid import NearestCentroid
from sklearn.ensemble import RandomForestClassifier
from sklearn import svm

from os.path import join
import numpy as np
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from scipy.stats import multivariate_normal
import numpy.random as npr
import numpy as np

class Particle:
    def __init__(self):
        self.pose, self.weight = Pose2D(), 1.0

    def update(self, control, cov_c, position, cov_p):
        pose = [pose.x + control[0], pose.y + control[1], pose.theta + control[2]]
        
        pose = multivariate_normal.rvs(mean=np.asarray(pose).flatten(), cov=cov_c).reshape(2, 1)
        
        self.weight = multivariate_normal.pdf(pose, mean=position, cov=cov_p)
        pass


class ParticleFilter:
    def __init__(self, num_particles):
        self.particles = [Particle() for _ in range(num_particles)]
        pass

    def update(self, control, cov_c, position, cov_p):
        for particle in self.particles:
            particle.update(control, cov_c, position, cov_p)
        
        weights = np.array([particle.weight for particle in self.particles])
        
        N_eff = 1 / sum(np.power(weights,2))
        if N_eff < len(self.particles) / 2:
            sample_particles = []
            for i in npr.choice(range(len(self.particles)), len(self.particles), p=weights):
                if self.particles[i] in sample_particles:
                    particle = copy.deepcopy(self.particles[i])
                else:
                    particle = self.particles[i]
                sample_particles.append(particle)
                pass
            self.particles = sample_particles
        pass


class BayesianPixelClassifier:
    def __init__(self):
        self.classifier = RandomForestClassifier(n_estimators=10) #GaussianNB()
        pass

    def train(self, image, mask):
        if image.shape[:2] != mask.shape[:2]:
            return
        width, height = image.shape[:2]
        self.classifier.fit(image.reshape(width*height, 3).tolist(), mask.reshape(width*height).tolist())
        return

    def predict(self, image, invert=True):
        width, height = image.shape[:2]
        img = image.reshape((width*height, 3))
        result = self.classifier.predict(img.tolist()).reshape((width, height))
        return result


class RobotFinder:
    def __init__(self, training_images, response_images, test_name):
        self.classifier = BayesianPixelClassifier()
        self.detector = cv2.SimpleBlobDetector()
        self.bridge = CvBridge()
        
        package = join(rospkg.RosPack().get_path('sailing_stones'), 'cfg')
        for img_name, mask_name in zip(training_images, response_images):
            img = cv2.imread(join(package, img_name))
            mask = cv2.imread(join(package, mask_name), cv2.CV_LOAD_IMAGE_GRAYSCALE)
            print "Train"
            if img is None or mask is None:
                continue
            self.classifier.train(img, mask)
        print "Test %s" % join(package, test_name)
        test = cv2.imread(join(package, test_name))
        print "Classify"
        out = self.classifier.predict(test)
        print "Output %s" % join(package, "output.jpg")
        cv2.imwrite(join(package, "output.jpg"), out)
        pass
    
    def odom_callback(self, message):
        pass

    def image_callback(self, message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError, e:
            print e

        cv2.cvtColor(cv_image, cv_image, cv2.COLOR_BGR2RGB)

        grey = self.classifier.predict(cv_image)

        keypoints = self.detector.detect(grey)

        im_with_keypoints = cv2.drawKeypoints(grey, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        pass

def main(args):
    rospy.init_node('find_robot')
    arg_defaults = {
        'training_images' : [
                             'template/IMG_1.jpg',
                             'template/IMG_2.jpg',
                             'template/IMG_3.jpg',
                             'template/IMG_4.jpg',
                             'template/IMG_0293.jpg'],
        'response_images' : [
                             'template/IMG_1_mask.jpg',
                             'template/IMG_2_mask.jpg',
                             'template/IMG_3_mask.jpg',
                             'template/IMG_4_mask.jpg',
                             'template/IMG_0293_mask.jpg'],
        'test_name' : 'IMG_0293.jpg'
    }
    args = updateArgs(arg_defaults)
    RobotFinder(**args)
    print "Done"
    return
    #try:
    #    rospy.spin()
    #except rospy.ROSInterruptException, e:
    #    print e


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
