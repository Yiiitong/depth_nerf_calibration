from __future__ import print_function
from __future__ import division
import os
import rospy
import rospkg
import numpy as np
from tf2_ros import TransformListener, Buffer, LookupException, ExtrapolationException, ConnectivityException
from tf.transformations import quaternion_multiply, quaternion_from_euler, rotation_matrix, quaternion_from_matrix, quaternion_matrix
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from scipy.spatial.transform import Rotation as R

try:
    from python_qt_binding.QtGui import QWidget, QListWidgetItem, QLabel
except ImportError:
    try:
        from python_qt_binding.QtWidgets import QWidget, QListWidgetItem, QLabel, QVBoxLayout
    except:
        raise ImportError('Could not import QWidgets')

calibrate = False

calib = []

class RqtCalibrationEvaluator(Plugin):
    def __init__(self, context):

        super(RqtCalibrationEvaluator, self).__init__(context)

        self.checkerboard_measured = np.loadtxt("/home/yitong/catkin_ws/save_points_pointer_tip_np.txt")
        self.checkerboard_picked   = np.array([[0,0,0,1000],
                                                [0,175,0,1000],
                                                [175,175,0,1000],
                                                [175,0,0,1000],
                                                [35,35,0,1000],
                                                [35,140,0,1000],
                                                [140,140,0,1000],
                                                [140,35,0,1000],
                                                [70,70,0,1000],
                                                [70,105,0,1000],
                                                [105,105,0,1000],
                                                [105,70,0,1000]])/1000


        self.checkerboard_measured = np.concatenate([self.checkerboard_measured,np.ones((12,1))],-1)

        # Give QObjects reasonable names
        self.setObjectName('CalibrationEvaluator')

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

        # Create QWidget
        self._widget = QWidget()
        self._infoWidget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_easy_handeye'), 'resource', 'rqt_handeye_evaluator.ui')
        ui_info_file = os.path.join(rospkg.RosPack().get_path('rqt_easy_handeye'), 'resource', 'rqt_handeye_info.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        loadUi(ui_info_file, self._infoWidget)
        self._widget.layout().insertWidget(0, self._infoWidget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtHandeyeCalibrationUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._infoWidget.calibNameLineEdit.setText(rospy.get_namespace())
        if rospy.get_param('eye_on_hand', False):
            self._infoWidget.calibTypeLineEdit.setText("eye on hand")
        else:
            self._infoWidget.calibTypeLineEdit.setText("eye on base")
        self._infoWidget.trackingBaseFrameLineEdit.setText(rospy.get_param('tracking_base_frame', 'optical_origin'))
        self._infoWidget.trackingMarkerFrameLineEdit.setText(rospy.get_param('tracking_marker_frame', 'optical_target'))
        self._infoWidget.robotBaseFrameLineEdit.setText(rospy.get_param('robot_base_frame', 'base_link'))
        self._infoWidget.robotEffectorFrameLineEdit.setText(rospy.get_param('robot_effector_frame', 'tool0'))

        self.output_label = self._widget.label_message
        self.output_label.setText('Waiting for samples...')

        self._widget.pushButton_reset.clicked.connect(self.reset)

        self.is_eye_on_hand = rospy.get_param('~eye_on_hand')
        self.robot_base_frame = rospy.get_param('~robot_base_frame')
        self.robot_effector_frame = rospy.get_param('~robot_effector_frame')

        self.tracking_measurement_frame = rospy.get_param('~tracking_marker_frame')
        if self.is_eye_on_hand:
            self.robot_measurement_frame = self.robot_base_frame
        else:
            self.robot_measurement_frame = self.robot_effector_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.tick)
        self.update_timer.start(500)

        self.last_robot_transform = None  # used to see if we are in a steady state

        self.measurement_transforms = []  # used to measure the quality of the calibration: should have the same value
        self.robot_transforms = []  # used to determine when to sample: we wait for a steady state that has not been sampled yet

        self._widget.show()

    def tick(self):
        # wait for steady state to avoid problems with lag
        # if next to a copy already in the dataset, tell to move
        # if could not sample transform, tell how old the last one is
        # show average error; show dataset in 3D?

        if not calibrate:
            try:
                
                GT_tf = self.tf_buffer.lookup_transform(self.robot_base_frame, "marker",
                                                                    rospy.Time(),
                                                                    rospy.Duration.from_sec(0.2))
                rot = quaternion_matrix([GT_tf.transform.rotation.x,
                                        GT_tf.transform.rotation.y,
                                        GT_tf.transform.rotation.z,
                                        GT_tf.transform.rotation.w])
                trans = np.array([GT_tf.transform.translation.x,GT_tf.transform.translation.y,GT_tf.transform.translation.z])
                GT_pose = rot
                GT_pose[:3,3] = trans

                
                new_robot_transform = self.tf_buffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame,
                                                                    rospy.Time(),
                                                                    rospy.Duration.from_sec(0.2))
                new_measurement_transform = self.tf_buffer.lookup_transform(self.robot_measurement_frame, # optical origin
                                                                            self.tracking_measurement_frame, # board
                                                                            rospy.Time(),
                                                                            rospy.Duration.from_sec(0.2))

                rotation_corner = quaternion_matrix(np.array([new_measurement_transform.transform.rotation.x,
                                                            new_measurement_transform.transform.rotation.y,
                                                            new_measurement_transform.transform.rotation.z,
                                                            new_measurement_transform.transform.rotation.w]))
                rotation_corner = rotation_corner[:3, :3]
                translate_corner = np.array([new_measurement_transform.transform.translation.x,
                                            new_measurement_transform.transform.translation.y,
                                            new_measurement_transform.transform.translation.z])

                GT_rotation = GT_pose[:3, :3]

                R_y_90 = rotation_matrix( -90 / 180.0 * np.pi, [0, 1, 0])
                R_z_180 = rotation_matrix( 180 / 180.0 * np.pi, [0, 0, 1]) #180

                pose_trans = np.matmul(R_z_180[:3, :3], R_y_90[:3, :3])


                rotation_corner = np.matmul(rotation_corner, pose_trans)

                translate_error = np.linalg.norm(translate_corner - (GT_pose[:3, 3]))*1000
                rotation_error = np.degrees(np.arccos((np.trace(np.matmul(rotation_corner.T, GT_rotation)) - 1) / 2)) # / np.pi * 180

                new_gt_marker = np.identity(4)
                new_gt_marker[:3, :3] = GT_rotation
                new_gt_marker[:3, 3] = GT_pose[:3, 3]

                gt_marker_as_list = [each_elem for each_elem in GT_pose[:3,3]] + [each_elem for each_elem in quaternion_from_matrix(new_gt_marker)]
                print('gt_marker_pose', gt_marker_as_list)

                rospy.logwarn('Rotation error: {0:02f} and translation error: {1:02f}'.format(rotation_error,
                                                                                translate_error))

                print('Rotation error:', rotation_error, 'and translation error: ', translate_error)


                if new_robot_transform is None:
                    rospy.logwarn('Could not sample transform between {} and {}'.format(self.robot_base_frame,
                                                                                        self.robot_effector_frame))
                if new_measurement_transform is None:
                    rospy.logwarn('Could not sample transform between {} and {}'.format(self.robot_measurement_frame,
                                                                                        self.tracking_measurement_frame))
                if self.last_robot_transform is None:
                    self.last_robot_transform = new_robot_transform
                    self.updateUI()
                    rospy.loginfo('Sampled first transform')
                    return
                if RqtCalibrationEvaluator.transform_too_far(new_robot_transform, self.last_robot_transform,
                                                            absolute_tolerance=0.0001):
                    self.last_robot_transform = new_robot_transform
                    msg = 'Waiting for steady state'
                    rospy.loginfo(msg)
                    self.output_label.setText(msg)
                    return

                if self.robot_transform_is_too_close_to_previous_sample(new_robot_transform, absolute_tolerance=0.003):
                    self.updateUI()
                    rospy.loginfo('Now we have {} samples\ntoo close to an old pose, move around!'.format(
                        len(self.measurement_transforms)))
                    self.output_label.setText('Too close to an old pose, move around!')
                    return

                self.robot_transforms.append(new_robot_transform)
                self.measurement_transforms.append(new_measurement_transform)
                rospy.loginfo('Appending transform; we got {} now'.format(len(self.measurement_transforms)))
                self.updateUI()
                # TODO: provide feedback if the data is sufficient
            except (LookupException, ExtrapolationException, ConnectivityException) as e:
                msg = 'Could not sample pose!'
                rospy.loginfo(msg)
                rospy.logerr(e)
                self.output_label.setText(msg)

        elif calibrate:

            try:
                
                marker_to_base = self.tf_buffer.lookup_transform('optical_origin','marker',
                                                                    rospy.Time(),
                                                                    rospy.Duration.from_sec(0.2))

                marker_to_base_matrix = tf_to_matrix(marker_to_base)

                ee_to_base = self.tf_buffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame,
                                                                    rospy.Time(),
                                                                    rospy.Duration.from_sec(0.2))
                ee_to_base_matrix = tf_to_matrix(ee_to_base)

                board_to_camera = self.tf_buffer.lookup_transform(rospy.get_param('~tracking_base_frame'), #camera
                                                                self.tracking_measurement_frame, #board
                                                                rospy.Time(),
                                                                rospy.Duration.from_sec(0.2))
                board_to_camera_matrix = tf_to_matrix(board_to_camera)


                R_y_90 = rotation_matrix( - 90 / 180.0 * np.pi, [0, 1, 0]) # -90
                R_z_180 = rotation_matrix( 180 / 180.0 * np.pi, [0, 0, 1]) # 180
                pose_trans =  np.matmul(R_z_180[:3, :3], R_y_90[:3, :3])
                board_to_camera_matrix[:3,:3] = np.matmul(board_to_camera_matrix[:3,:3],pose_trans)


                ee_to_board = np.matmul(np.linalg.inv(marker_to_base_matrix),ee_to_base_matrix)
                camera_to_ee = np.linalg.inv(np.matmul(board_to_camera_matrix,ee_to_board))
                # print(camera_to_ee[:3,3],quaternion_from_matrix(camera_to_ee))

                if len(calib)>10:
                    calib.pop()
                as_list = [each_elem for each_elem in camera_to_ee[:3,3]] + [each_elem for each_elem in quaternion_from_matrix(camera_to_ee)]
                
                calib.insert(0,as_list)
                calib_as_np = np.array(calib).mean(0)
                print(calib_as_np)

                ####### add this evaluation:
                camera_to_base = tf_to_matrix(self.tf_buffer.lookup_transform(self.robot_base_frame, rospy.get_param('~tracking_base_frame'),
                                                                    rospy.Time(),
                                                                    rospy.Duration.from_sec(0.2)))

                board_to_base = np.matmul(camera_to_base,board_to_camera_matrix)
                picked_to_base = np.matmul(board_to_base,self.checkerboard_picked.T).T

                diff = np.abs(self.checkerboard_measured - picked_to_base)*1000

                print('diff:', diff)

                '''

                rospy.logwarn('Rotation error: {0:02f} and translation error: {1:02f}'.format(rotation_error,
                                                                                translate_error))

                print('Rotation error:', rotation_error, 'and translation error: ', translate_error)

                if base_to_ee is None:
                    rospy.logwarn('Could not sample transform between {} and {}'.format(self.robot_base_frame,
                                                                                        self.robot_effector_frame))
                if new_measurement_transform is None:
                    rospy.logwarn('Could not sample transform between {} and {}'.format(self.robot_measurement_frame,
                                                                                        self.tracking_measurement_frame))
                if self.last_robot_transform is None:
                    self.last_robot_transform = base_to_ee
                    self.updateUI()
                    rospy.loginfo('Sampled first transform')
                    return
                if RqtCalibrationEvaluator.transform_too_far(base_to_ee, self.last_robot_transform,
                                                            absolute_tolerance=0.0001):
                    self.last_robot_transform = base_to_ee
                    msg = 'Waiting for steady state'
                    rospy.loginfo(msg)
                    self.output_label.setText(msg)
                    return

                if self.robot_transform_is_too_close_to_previous_sample(base_to_ee, absolute_tolerance=0.003):
                    self.updateUI()
                    rospy.loginfo('Now we have {} samples\ntoo close to an old pose, move around!'.format(
                        len(self.measurement_transforms)))
                    self.output_label.setText('Too close to an old pose, move around!')
                    return

                self.robot_transforms.append(base_to_ee)
                self.measurement_transforms.append(new_measurement_transform)
                rospy.loginfo('Appending transform; we got {} now'.format(len(self.measurement_transforms)))
                self.updateUI()
                # TODO: provide feedback if the data is sufficient

                '''
            except (LookupException, ExtrapolationException, ConnectivityException) as e:
                msg = 'Could not sample pose!'
                rospy.loginfo(msg)
                rospy.logerr(e)
                self.output_label.setText(msg)

    def reset(self):
        self.robot_transforms = []
        self.measurement_transforms = []
        self.updateUI()

    def updateUI(self):
        self._widget.spinBox_samples.setValue(len(self.measurement_transforms))
        if len(self.measurement_transforms) > 2:
            def translation_from_msg(msg):
                t = msg.transform.translation
                return t.x, t.y, t.z

            translations = [translation_from_msg(t) for t in self.measurement_transforms]
            translations_np = np.array(translations)
            translations_avg = translations_np.mean(axis=0)
            translations_from_avg = translations_np - translations_avg
            translations_max_divergence = np.max(translations_from_avg)
            rospy.loginfo("Maximum divergence: {}".format(translations_max_divergence))

            self._widget.doubleSpinBox_error.setEnabled(True)
            self._widget.doubleSpinBox_error.setValue(translations_max_divergence.max())
        else:
            self._widget.doubleSpinBox_error.setValue(0)
            self._widget.doubleSpinBox_error.setEnabled(False)

    @staticmethod
    def transform_to_concatenated_translation_quaternion(transform):
        tr = transform.transform.translation
        quat = transform.transform.rotation
        return np.array([tr.x, tr.y, tr.z, quat.x, quat.y, quat.z, quat.w])

    def robot_transform_is_too_close_to_previous_sample(self, new_robot_transform, absolute_tolerance):
        # TODO: use a meaningful metric
        posevec = RqtCalibrationEvaluator.transform_to_concatenated_translation_quaternion(new_robot_transform)
        for t in reversed(self.robot_transforms):
            old_posevec = RqtCalibrationEvaluator.transform_to_concatenated_translation_quaternion(t)
            if np.allclose(posevec, old_posevec, atol=absolute_tolerance):
                return True
        return False

    @staticmethod
    def transform_too_far(t1, t2, absolute_tolerance):
        # TODO: use a meaningful metric
        return not np.allclose(RqtCalibrationEvaluator.transform_to_concatenated_translation_quaternion(t1),
                               RqtCalibrationEvaluator.transform_to_concatenated_translation_quaternion(t2),
                               atol=absolute_tolerance)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

def tf_to_matrix(tf_pose):

    tf_matrix = quaternion_matrix([tf_pose.transform.rotation.x,
                                    tf_pose.transform.rotation.y,
                                    tf_pose.transform.rotation.z,
                                    tf_pose.transform.rotation.w])
    tf_matrix[:3,3] = np.array([tf_pose.transform.translation.x,
                                tf_pose.transform.translation.y,
                                tf_pose.transform.translation.z])

    return tf_matrix

