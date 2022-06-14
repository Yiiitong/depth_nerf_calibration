#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import cv2
import os
import math
import numpy as np
from copy import deepcopy
from collections import OrderedDict

from tf.transformations import quaternion_multiply, quaternion_from_euler, rotation_matrix, quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point

from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

import rospy
import rospkg

from itertools import chain
try:
    from itertools import izip as zip
except ImportError: # will be 3.x series
    pass

from easy_handeye.handeye_client import HandeyeClient
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QListWidgetItem, QLabel, \
                                        QComboBox, QHBoxLayout, QVBoxLayout, \
                                        QPushButton, QLineEdit, QFormLayout, \
                                        QPlainTextEdit

# rospy.init_node('point_coordinate_gui', anonymous=True)
point_sample_list = []

robot_base_frame = 'optical_origin'
robot_effector_frame = 'pointer_tip'
calibration = False

if not calibration:
    robot_effector_frame = 'tool_tip'
else:
    robot_effector_frame = "pointer_tip"

def format_sample(sample):
    if calibration:
        sample = np.array(sample[:3,3])
    x, y, z = sample[0], sample[1], sample[2]
    return 'translation: [{:+.6f}, {:+.6f}, {:+.6f}]\n'.format(x, y, z)

class GetPointcoordinate(Plugin):
    
    def __init__(self, context):
        super(GetPointcoordinate, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GetPointcoordinate')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        ## GUI
        if True:
            # Create QWidgets
            self._widget = QWidget()
            self._infoWidget = QWidget()

            # Get path to UI file which should be in the "resource" folder of this package
            ui_file = os.path.join(rospkg.RosPack().get_path('rqt_easy_handeye'), 'resource', 'rqt_pointcoordinate.ui')
            ui_info_file = os.path.join(rospkg.RosPack().get_path('rqt_easy_handeye'), 'resource', 'rqt_pointcoordinate_info.ui')
            # Extend the widget with all attributes and children from UI file
            loadUi(ui_file, self._widget)
            loadUi(ui_info_file, self._infoWidget)
            self._widget.horizontalLayout_infoAndActions.insertWidget(0, self._infoWidget)

            # Give QObjects reasonable names
            self._widget.setObjectName('GetPointcoordinateGUI')
            self._widget.setWindowTitle('GetPointcoordinateGUI')

            # Add widget to the user interface
            context.add_widget(self._widget)

            self._infoWidget.robotBaseFrameLineEdit.setText(robot_base_frame)
            self._infoWidget.robotEffectorFrameLineEdit.setText(robot_effector_frame)
            self._widget.takeButton.clicked[bool].connect(self.take_point_sample)
            self._widget.removeButton.clicked[bool].connect(self.handle_remove_sample)
            self._widget.saveButton.clicked[bool].connect(self.handle_save_points)

            self._widget.removeButton.setEnabled(False)

            self._widget.saveButton.setEnabled(False)


        self.joint_limits = np.array([90] * 5 + [180] + [350])

        self.tfBuffer = Buffer(cache_time=rospy.Time(1000))
        self.tfListener = TransformListener(self.tfBuffer)

        self.cv_bridge = CvBridge()
    
    def _display_sample_list(self, sample_list):
        self._widget.sampleListWidget.clear()

        for i in range(len(point_sample_list)):
            formatted_sample = format_sample(point_sample_list[i])
            self._widget.sampleListWidget.addItem(
                '{}) \n point: \n {} \n'.format(i + 1, formatted_sample))
        self._widget.sampleListWidget.setCurrentRow(len(sample_list) - 1)
        self._widget.removeButton.setEnabled(len(sample_list) > 0)
    

    def take_point_sample(self):
        tfBuffer = Buffer(cache_time=rospy.Time(1000))
        tfListener = TransformListener(tfBuffer)
        
        source_frame = robot_base_frame
        target_frame = robot_effector_frame
        opt = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0.0), rospy.Duration(10))
        R_ee = quaternion_matrix(np.array([opt.transform.rotation.x, opt.transform.rotation.y, opt.transform.rotation.z, opt.transform.rotation.w]))[:3, :3]
        t_ee = np.array([opt.transform.translation.x, opt.transform.translation.y, opt.transform.translation.z])
        T_ee = np.eye(4, 4)
        T_ee[:3, :3] = R_ee
        T_ee[:3, 3]  = t_ee
        
        ee_to_pointer = np.identity(4)

        T_pointer = np.matmul(T_ee,ee_to_pointer)
        if not calibration:  T_pointer = np.array(T_pointer[:3,3])

        # print(T_pointer)
        # print(T_ee)
        # print(np.matmul(np.linalg.inv(T_ee),T_pointer))
        
        point_sample_list.append(T_pointer)
        # rospy.loginfo("ee_pose:"+ str(T_marker) )
        self._display_sample_list(point_sample_list)
        print(T_pointer)
        self._widget.saveButton.setEnabled(True)


    def handle_remove_sample(self):
        index = self._widget.sampleListWidget.currentRow()
        point_sample_list.pop(-1)
        self._display_sample_list(point_sample_list)
        
        self._widget.saveButton.setEnabled(True)
    
    def handle_save_points(self):
        # fd = os.open("/home/yitong/catkin_ws/save_points.txt",os.O_RDWR|os.O_CREAT)
        print('\n')
        print('Sampled points: \n')

        for i in range(len(point_sample_list)):
            print(point_sample_list[i])            
            # ret = os.write(fd,str(point_sample_list[i]))
            # ret = os.write(fd,'\n')

        if calibration:
            point_array = np.concatenate(point_sample_list,0)
            np.savetxt("/home/yitong/catkin_ws/save_points_np_calib.txt",point_array)
        
        else:
            point_array = np.stack(point_sample_list,0)
            np.savetxt("/home/yitong/catkin_ws/save_points_pointer_tip_np.txt",point_array)
        

            
        # os.close(fd)
        self._widget.saveButton.setEnabled(True)



        
    
