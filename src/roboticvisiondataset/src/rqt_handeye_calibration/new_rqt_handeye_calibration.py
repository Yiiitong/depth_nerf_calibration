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

from scipy.spatial.transform import Rotation as R

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QListWidgetItem, QLabel, \
                                        QComboBox, QHBoxLayout, QVBoxLayout, \
                                        QPushButton, QLineEdit, QFormLayout, \
                                        QPlainTextEdit
# rospy.init_node('new_rqt_handeye_calibration', anonymous=True)

def tf_to_matrix(tf_pose):

    tf_matrix = quaternion_matrix([tf_pose.transform.rotation.x,
                                    tf_pose.transform.rotation.y,
                                    tf_pose.transform.rotation.z,
                                    tf_pose.transform.rotation.w])
    tf_matrix[:3,3] = np.array([tf_pose.transform.translation.x,
                                tf_pose.transform.translation.y,
                                tf_pose.transform.translation.z])

    return tf_matrix

class New_HandeyeCalibration(Plugin):
    AVAILABLE_ALGORITHMS = OrderedDict([
            ('Tsai-Lenz', cv2.CALIB_HAND_EYE_TSAI),
            ('Park', cv2.CALIB_HAND_EYE_PARK),
            ('Horaud', cv2.CALIB_HAND_EYE_HORAUD),
            ('Andreff', cv2.CALIB_HAND_EYE_ANDREFF),
            ('Daniilidis', cv2.CALIB_HAND_EYE_DANIILIDIS)])
    
    def __init__(self, context):
        super(New_HandeyeCalibration, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('New_HandeyeCalibration')
        self.samples = []
        self.image_num = 0

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
            self._widget_layout = QVBoxLayout()
            self._widget.setLayout(self._widget_layout)

            self._calibration_algorithm_combobox = QComboBox()
            self._calibration_algorithm_layout = QHBoxLayout()
            self._calibration_algorithm_layout.insertWidget(0, QLabel('Calibration algorithm: '))
            self._calibration_algorithm_layout.insertWidget(1, self._calibration_algorithm_combobox)
            self._widget.layout().insertLayout(-1, self._calibration_algorithm_layout)

            self._boxTrackingBaseFrame = QLineEdit()
            self._layoutTrackingBaseFrame = QHBoxLayout()
            self._layoutTrackingBaseFrame.insertWidget(0, QLabel('Tracking base frame: '))
            self._layoutTrackingBaseFrame.insertWidget(1, self._boxTrackingBaseFrame)
            self._widget.layout().insertLayout(-1, self._layoutTrackingBaseFrame)

            self._boxTrackingMarkerFrame = QLineEdit()
            self._layoutTrackingMarkerFrame = QHBoxLayout()
            self._layoutTrackingMarkerFrame.insertWidget(0, QLabel('Tracking marker frame: '))
            self._layoutTrackingMarkerFrame.insertWidget(1, self._boxTrackingMarkerFrame)
            self._widget.layout().insertLayout(-1, self._layoutTrackingMarkerFrame)

            self._boxRobotBaseFrame = QLineEdit()
            self._layoutRobotBaseFrame = QHBoxLayout()
            self._layoutRobotBaseFrame.insertWidget(0, QLabel('Calibration base frame: '))
            self._layoutRobotBaseFrame.insertWidget(1, self._boxRobotBaseFrame)
            self._widget.layout().insertLayout(-1, self._layoutRobotBaseFrame)

            self._boxRobotEffectorFrame = QLineEdit()
            self._layoutRobotEffectorFrame = QHBoxLayout()
            self._layoutRobotEffectorFrame.insertWidget(0, QLabel('Calibration effector frame: '))
            self._layoutRobotEffectorFrame.insertWidget(1, self._boxRobotEffectorFrame)
            self._widget.layout().insertLayout(-1, self._layoutRobotEffectorFrame)

            self._boxCameraTopic = QLineEdit()
            self._layoutCameraTopic = QHBoxLayout()
            self._layoutCameraTopic.insertWidget(0, QLabel('Camera topic: '))
            self._layoutCameraTopic.insertWidget(1, self._boxCameraTopic)
            self._widget.layout().insertLayout(-1, self._layoutCameraTopic)

            self._boxAngleDelta = QLineEdit()
            self._layoutAngleDelta = QHBoxLayout()
            self._layoutAngleDelta.insertWidget(0, QLabel('Angle delta: '))
            self._layoutAngleDelta.insertWidget(1, self._boxAngleDelta)
            self._widget.layout().insertLayout(-1, self._layoutAngleDelta)

            self._boxTranslationDelta = QLineEdit()
            self._layoutTranslationDelta = QHBoxLayout()
            self._layoutTranslationDelta.insertWidget(0, QLabel('Translation delta: '))
            self._layoutTranslationDelta.insertWidget(1, self._boxTranslationDelta)
            self._widget.layout().insertLayout(-1, self._layoutTranslationDelta)

            self._btnCalibrate = QPushButton(text="calibrate")
            self._widget.layout().insertWidget(-1, self._btnCalibrate)

            self._btnSample = QPushButton(text="sample")
            self._widget.layout().insertWidget(-1, self._btnSample)

            self._editConsole = QPlainTextEdit()
            self._widget.layout().insertWidget(-1, self._editConsole)

            # Give QObjects reasonable names
            self._widget.setObjectName('New_HandeyeCalibrationHandeyeCalibrationGUI')
            self._widget.setWindowTitle('New_HandeyeCalibrationHandeyeCalibrationGUI')

            # Add widget to the user interface
            context.add_widget(self._widget)

            for i, a in enumerate(self.AVAILABLE_ALGORITHMS):
                self._calibration_algorithm_combobox.insertItem(i, a)

            self._calibration_algorithm_combobox.setCurrentIndex(0)

            namespace = rospy.get_name() + "/"
            self._boxTrackingBaseFrame.setText(rospy.get_param(namespace + 'tracking_base_frame'))
            self._boxTrackingMarkerFrame.setText(rospy.get_param(namespace + "tracking_marker_frame"))
            self._boxRobotBaseFrame.setText(rospy.get_param(namespace + "robot_base_frame"))
            self._boxRobotEffectorFrame.setText(rospy.get_param(namespace + "robot_effector_frame"))
            self._boxCameraTopic.setText(rospy.get_param(namespace + "camera_topic"))
            self._boxAngleDelta.setText(str(rospy.get_param(namespace + "angle_delta")))
            self._boxTranslationDelta.setText(str(rospy.get_param(namespace + "translation_delta")))


            self._btnSample.clicked[bool].connect(self.sample)

            self._btnSample.setEnabled(True)

            self._btnCalibrate.clicked[bool].connect(self.calibrate)

            self._btnCalibrate.setEnabled(False)

            self._editConsole.setPlainText("Waiting for camera topics...")
    
        
        # if not move_group_namespace.endswith('/'):
        #     move_group_namespace = move_group_namespace + '/'
        # if move_group_namespace != '/':
        #     self.mgc = MoveGroupCommander(move_group_name, robot_description=move_group_namespace+'robot_description', ns=move_group_namespace)
        # else:
        #     self.mgc = MoveGroupCommander(move_group_name)
        # self.mgc.set_planner_id("RRTConnectkConfigDefault")  # TODO: this is only needed for the UR5
        # self.mgc.set_max_velocity_scaling_factor(max_velocity_scaling)
        # self.mgc.set_max_acceleration_scaling_factor(max_acceleration_scaling)

        self.joint_limits = np.array([90] * 5 + [180] + [350])

        self.tfBuffer = Buffer(cache_time=rospy.Time(1000))
        self.tfListener = TransformListener(self.tfBuffer)

        self.tfBuffer2 = Buffer(cache_time=rospy.Time(1000))
        self.tfListener2 = TransformListener(self.tfBuffer2)

        self.cv_bridge = CvBridge()

        camera_topic = self._boxCameraTopic.text()
        self.image = None
        self.image_sub = rospy.Subscriber(camera_topic + "/image_raw", Image, self.image_callback)

        camera_topic = self._boxCameraTopic.text()
        self.camera_info = None
        self.camera_info_sub = rospy.Subscriber(camera_topic + "/camera_info", CameraInfo, self.camera_info_callback)

        
    


    def sample(self):
        source_frame = self._boxTrackingBaseFrame.text()
        target_frame = self._boxTrackingMarkerFrame.text()
        base_frame = self._boxRobotBaseFrame.text()
        end_effector_frame = self._boxRobotEffectorFrame.text()
        #opt = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time().now(), rospy.Duration(10))
        #X_start = np.array([opt.transform.translation.x, opt.transform.translation.y, opt.transform.translation.z])
        
        # While not found N images
        num_images = 15
        
        
        if self.image_num < num_images:
            # Sleep
            rospy.sleep(1.0)

            # Get current marker pose
            opt = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time().now(), rospy.Duration(10))
            R_marker = quaternion_matrix(np.array([opt.transform.rotation.x, opt.transform.rotation.y, opt.transform.rotation.z, opt.transform.rotation.w]))[:3, :3]
            t_marker = np.array([opt.transform.translation.x, opt.transform.translation.y, opt.transform.translation.z])
            T_marker = np.eye(4, 4)
            T_marker[:3, :3] = R_marker
            T_marker[:3, 3] = t_marker

            R_y_90 = rotation_matrix( - 90 / 180.0 * np.pi, [0, 1, 0])
            R_z_180 = rotation_matrix( 180 / 180.0 * np.pi, [0, 0, 1])
            pose_trans =  np.matmul(R_z_180[:3, :3], R_y_90[:3, :3])
            T_marker[:3,:3] = np.matmul(T_marker[:3,:3],pose_trans)

            print('Marker:',t_marker)

            # Get current effector pose
            effector_pose = self.tfBuffer2.lookup_transform(base_frame, end_effector_frame, rospy.Time().now(), rospy.Duration(10))
            R_effector = quaternion_matrix(np.array([effector_pose.transform.rotation.x, effector_pose.transform.rotation.y, effector_pose.transform.rotation.z, effector_pose.transform.rotation.w]))[:3, :3]
            t_effector = np.array([effector_pose.transform.translation.x, effector_pose.transform.translation.y, effector_pose.transform.translation.z])
            T_effector = np.eye(4, 4)
            T_effector[:3, :3] = R_effector
            T_effector[:3, 3] = t_effector   

            R_x_90 = rotation_matrix( 90 / 180.0 * np.pi, [1, 0, 0])
            R_z_180 = rotation_matrix( 180 / 180.0 * np.pi, [0, 0, 1])
            pose_trans =  np.linalg.inv(np.matmul(R_z_180[:3, :3], R_x_90[:3, :3]))
            T_effector[:3,:3] = np.matmul(T_effector[:3,:3], pose_trans)
            print('End-effector:', t_effector)                 

            # Store
            self.samples.append({'T_marker': T_marker, 'T_effector': T_effector, 'image': self.image})
            self.image_num += 1  
            print('Sampled '+str(self.image_num))                 

        else:
            print('Have enough samples, ready to calibrate')

        
        

    def calibrate(self):
        
        # Do calibration
        print(self.samples)
        R_gripper2base = [x['T_effector'][:3, :3] for x in self.samples]
        t_gripper2base = [x['T_effector'][:3, 3] for x in self.samples]
        R_target2cam = [x['T_marker'][:3, :3] for x in self.samples]
        t_target2cam = [x['T_marker'][:3, 3] for x in self.samples]
        method = list(self.AVAILABLE_ALGORITHMS.values())[self._calibration_algorithm_combobox.currentIndex()]
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=method)
        T_handeye = np.eye(4, 4)
        T_handeye[:3, :3] = R_cam2gripper
        T_handeye[:3, 3] = t_cam2gripper[:, 0]
        self._editConsole.setPlainText(str(T_handeye))

        print("as matrix")
        print(T_handeye)
        print("as quaternion with vector")
        print(R.from_matrix(R_cam2gripper).as_quat())
        print(t_cam2gripper[:, 0])
        np.savetxt('/home/yitong/hiwi_depth/extrinsic1.txt',T_handeye)

    def image_callback(self, msg):
        self.image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.check_init()
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.check_init()
    
    def check_init(self):
        if not self.image is None and not self.camera_info is None and not self._btnCalibrate.isEnabled():
            self._btnCalibrate.setEnabled(True)
            self._editConsole.setPlainText("Ready")

    def is_good_plan(self, plan):
        if len(plan.joint_trajectory.points) == 0:
            return False

        np_traj = np.array([p.positions for p in plan.joint_trajectory.points])
        if len(np_traj) == 0:
            raise ValueError
        np_traj_max_per_joint = np_traj.max(axis=0)
        np_traj_min_per_joint = np_traj.min(axis=0)
        abs_rot_per_joint = abs(np_traj_max_per_joint - np_traj_min_per_joint)
        abs_rot_per_joint = [math.degrees(j) for j in abs_rot_per_joint]

        return (abs_rot_per_joint < self.joint_limits).all()

    def get_lookat_matrix(position_vector, front_vector, up_vector):
        m1 = np.zeros([4, 4], dtype=np.float32)
        m2 = np.zeros([4, 4], dtype=np.float32)

        z = normalize_vector(-front_vector)
        x = normalize_vector(np.cross(up_vector, z))
        y = np.cross(z, x)

        m1[0, :3] = x
        m1[1, :3] = y
        m1[2, :3] = z
        m1[3, 3] = 1.0

        m2[0, 0] = m2[1, 1] = m2[2, 2] = 1.0
        m2[3, :3] = -position_vector
        m2[3, 3] = 1.0

        return np.matmul(m1, m2)
