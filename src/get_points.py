#!/usr/bin/env python
import sip

sip.setapi('QVariant', 2)

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtWebKit import *

import rospy, rospkg
from sensor_msgs.msg import Image

import sys, re, os
import yaml
from os.path import join, isfile, isdir

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

refPt = []

pointsworld = []
i = 0
n = 0
camerano = 1
brushregion = 0
cv_image1 = None
cv_image2 = None
cv_image3 = None
cv_image4 = None
cv_image5 = None
cv_image6 = None


class HomographyCalib:
    def __init__(self, image_topics, calibration_directory):
        self.bridge = CvBridge()

        # wait for slow topics
        rospy.sleep(rospy.Duration(0.5))

        published_topics = rospy.get_published_topics()

        # only interested in published topics
        self.img_topics = [topic for topic in image_topics if topic in published_topics]

        for img_topic in self.image_topics:
            rospy.Subscriber(img_topic, Image, self.image_callback, img_topic)

        self.current_point = 0
        self.points = [None, None, None, None]
        self.image = None
        self.mouse_position = None
        self.active_image = 0

        calib_dir = join(rospkg.RosPack().get_path('sailing_stones'), calibration_directory)
        if not isdir(calib_dir):
            os.makedirs(calib_dir)

        self.calib_dir = calibration_directory

        ##  GUI Initialization
        self.app = QApplication([])
        self.widget = QWidget()
        self.widget.setWindowTitle('Homography Calibration')
        self.widget.resize(800, 800)

        layout = QVBoxLayout()

        combobox = QComboBox(self.widget)
        for i, topic in self.img_topics:
            combobox.addItem(topic)
            combobox[topic].connect(self.select_camera)
        layout.addWidget(self.combobox)

        self.image_view = QLabel()
        self.image_view.mousePressEvent = self.click_detected
        self.image_view.mouseMoveEvent = self.mouse_move
        layout.addWidget(self.image_view)

        frame = QFrame()
        gridlayout = QGridLayout()

        self.inputs = [[QLineEdit('') for _ in range(2)] for _ in range(4)]

        for i in range(4):
            gridlayout.addWidget(QLabel('Point %i' % i), 0, i)

            x_value, y_value = self.inputs[i][0], self.inputs[i][1]
            x_value.setValidator(QDoubleValidator(-10000, 10000, 3))
            y_value.setValidator(QDoubleValidator(-10000, 10000, 3))

            gridlayout.addWidget(x_value, 1, i)
            gridlayout.addWidget(y_value, 2, i)

        frame.setLayout(layout)

        button = QPushButton('Reset Camera Keypoints')
        button.clicked.connect(self.reset_camera_keypoints)
        layout.addWidget(button)

        button = QPushButton('Reset Global Keypoints')
        button.clicked.connect(self.reset_world_keypoints)
        layout.addWidget(button)

        button = QPushButton('Calculate Homography')
        button.clicked.connect(self.calculate_homography)
        layout.addWidget(button)

        self.widget.setLayout(layout)

        # Show the window and run the app
        self.widget.show()
        self.app.exec_()
        pass

    def image_callback(self, msg, topic):

        index = self.image_topics.index(topic)
        if self.active_image != index:
            return

        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            return

        self.display_image()
        pass

    def display_image(self):
        if self.image is None:
            return

        img = self.image.copy()

        for point in self.points:
            if point is None:
                continue
            cv2.circle(img, point, 5, (0,0,255), 3)

        if self.mouse_position is not None:
            cv2.circle(img, point, 5, (0,255,0), 3)

        cv2.cvtColor(img, cv2.COLOR_BGR2RGB, img)
        q_image = QImage(img, img.shape[1], img.shape[0], QImage.Format_RGB888)
        self.image_view.setPixmap(QPixmap.fromImage(q_image))
        pass

    def show_warning(self, message):
        dialog = QMessageBox()
        dialog.setWindowTitle("Error!")
        dialog.setText(message)
        dialog.setStandardButtons(QMessageBox.Ok)
        dialog.exec_()
        pass

    def mouse_move(self, event):
        x = event.pos().x()
        y = event.pos().y()

        self.mouse_position = (x,y)
        pass

    def click_detected(self, event):
        x, y = event.pos().x(), event.pos().y()
        if self.current_point < 4:
            self.points[self.current_point] = (x, y)
            self.current_point += 1
        else:
            self.show_warning("Too many points selected thus far")
        pass

    def reset_world_keypoints(self):
        for i in range(len(self.inputs)):
            for j, lineEdit in enumerate(self.inputs[i]):
                lineEdit.setText('')
        pass

    def reset_camera_keypoints(self):
        self.points = [None, None, None, None]
        self.current_point = 0
        pass

    def calculate_homography(self):

        if self.current_point < 4:
            return self.show_warning("Selecting %i more calibration points" % (4 - self.current_point))

        isfloat = lambda str : str.replace('.','',1).isdigit()

        world_coords = []
        for index in range(len(self.inputs)):
            x_str = self.inputs[index][0].text()
            if not isfloat(x_str):
                return self.show_warning("X value of world coordinate point %i is invalid" % index)

            y_str = self.inputs[index][1].text()
            if not isfloat(y_str):
                return self.show_warning("Y value of world coordinate point %i is invalid" % index)

            world_coords.append((float(x_str), float(y_str)))

        M, mask = cv2.findHomography(np.array(self.points, dtype='float32'),
                                     np.array(world_coords, dtype='float32'), cv2.RANSAC, 5.0)

        image_topic = self.img_topics[self.current_point]

        true_index = re.findall(r'\d+', image_topic)[-1]

        calibration_file = join(self.calib_dir, 'cam%i.yaml' % true_index)

        data = yaml.load(open(calibration_file))
        data['instances'][0]['R'] = M.tolist()
        data['instances'][0]['P'] = self.points.tolist()

        with open(calibration_file, 'w') as yaml_file:
            yaml_file.write(yaml.dump(data, default_flow_style=False))
        pass

    def select_camera(self, i):
        self.active_image = i
        self.mouse_position = None
        self.reset_world_keypoints()
        self.reset_camera_keypoints()
        pass


def main(args):
    rospy.init_node('get_points')

    arg_defaults = {
        'image_topics' : ['/watcher1/image_raw', '/watcher2/image_raw', '/watcher3/image_raw', '/watcher4/image_raw',
                          '/watcher5/image_raw', '/watcher6/image_raw'],
        'calibration_directory' : 'cfg/test/'
    }
    args = updateArgs(arg_defaults)

    HomographyCalib(**args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException, e:
        print e
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
