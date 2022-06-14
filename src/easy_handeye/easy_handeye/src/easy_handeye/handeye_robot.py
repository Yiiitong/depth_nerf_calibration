from __future__ import print_function
from __future__ import division
from tf.transformations import quaternion_multiply, quaternion_from_euler, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Quaternion, PoseStamped, Point
from moveit_commander import MoveGroupCommander
import rospy
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import CameraInfo
import numpy as np
from itertools import chain
try:
    from itertools import izip as zip
except ImportError: # will be 3.x series
    pass
from copy import deepcopy
import math


class CalibrationMovements:
    def __init__(self, move_group_name, max_velocity_scaling, max_acceleration_scaling, angle_delta, translation_delta, move_group_namespace='/'):
        # self.client = HandeyeClient()  # TODO: move around marker when eye_on_hand, automatically take samples via trigger topic
        if not move_group_namespace.endswith('/'):
            move_group_namespace = move_group_namespace + '/'
        if move_group_namespace != '/':
            self.mgc = MoveGroupCommander(move_group_name, robot_description=move_group_namespace+'robot_description', ns=move_group_namespace)
        else:
            self.mgc = MoveGroupCommander(move_group_name)
        self.mgc.set_planner_id("RRTConnectkConfigDefault")  # TODO: this is only needed for the UR5
        self.mgc.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.mgc.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        self.start_pose = self.mgc.get_current_pose()
        self.joint_limits = [math.radians(90)] * 5 + [math.radians(180)] + [math.radians(350)]  # TODO: make param
        self.target_poses = []
        self.current_pose_index = None
        self.current_plan = None
        self.fallback_joint_limits = [math.radians(90)] * 4 + [math.radians(90)] + [math.radians(180)] + [
            math.radians(350)]
        if len(self.mgc.get_active_joints()) == 6:
            self.fallback_joint_limits = self.fallback_joint_limits[1:]
        self.angle_delta=angle_delta
        self.translation_delta=translation_delta

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer)

        # Useful for secondary input sources (e.g. programmable buttons on robot)
        # self.cam_info_subscriber = rospy.Subscriber("/rs_l515/color/camera_info", CameraInfo, self.take_sample)
        # self.camera_info = None

    # def take_sample(self, sample):
    #     self.camera_info = sample
    
    def set_and_check_starting_position(self):
        # sets the starting position to the current robot cartesian EE position and checks movement in all directions
        # TODO: make repeatable
        #  - save the home position and each target position as joint position
        #  - plan to each target position and back, save plan if not crazy
        #  - return true if can plan to each target position without going crazy
        self.start_pose = self.mgc.get_current_pose()
        self.target_poses = self._compute_poses_around_state(self.start_pose, self.angle_delta, self.translation_delta)
        self.current_pose_index = -1
        # ret = self._check_target_poses(self.joint_limits)
        ret = True
        
        if ret:
            rospy.loginfo("Set current pose as home")
            return True
        else:
            rospy.logerr("Can't calibrate from this position!")
            self.start_pose = None
            self.target_poses = None
            return False

    def select_target_pose(self, i):
        number_of_target_poses = len(self.target_poses)
        if 0 <= i < number_of_target_poses:
            rospy.loginfo("Selected pose {} for next movement".format(i))
            self.current_pose_index = i
            return True
        else:
            rospy.logerr("Index {} is out of bounds: there are {} target poses".format(i, number_of_target_poses))
            return False

    def plan_to_start_pose(self):
        # TODO: use joint position http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#planning-to-a-joint-goal
        rospy.loginfo("Planning to home pose")
        return self._plan_to_pose(self.start_pose)

    def plan_to_current_target_pose(self):
        # TODO: use joint position http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#planning-to-a-joint-goal
        i = self.current_pose_index
        rospy.loginfo("Planning to target pose {}".format(i))
        return self._plan_to_pose(self.target_poses[i])

    def execute_plan(self):
        if self.plan is None:
            rospy.logerr("No plan found!")
            return False
        if CalibrationMovements._is_crazy_plan(self.plan, self.fallback_joint_limits):
            rospy.logerr("Crazy plan found, not executing!")
            return False
        self.mgc.execute(self.plan)
        return True

    def _plan_to_pose(self, pose):
        self.mgc.set_start_state_to_current_state()
        self.mgc.set_pose_target(pose)
        ret = self.mgc.plan()
        if type(ret) is tuple:
            # noetic
            success, plan, planning_time, error_code = ret
        else:
            # melodic
            plan = ret
        if CalibrationMovements._is_crazy_plan(plan, self.joint_limits):
            rospy.logwarn("Planning failed")
            self.plan = None
            return False
        else:
            rospy.loginfo("Planning successful")
            self.plan = plan
            return True

    def _check_target_poses(self, joint_limits):
        if len(self.fallback_joint_limits) == 6:
            joint_limits = joint_limits[1:]
        for fp in self.target_poses:
            self.mgc.set_pose_target(fp)
            ret = self.mgc.plan()
            if type(ret) is tuple:
                # noetic
                success, plan, planning_time, error_code = ret
            else:
                # melodic
                plan = ret
            if len(plan.joint_trajectory.points) == 0 or CalibrationMovements._is_crazy_plan(plan, joint_limits):
                return False
        return True

    def _compute_poses_around_state(self, start_pose, angle_delta, translation_delta):
        opt = self.tfBuffer.lookup_transform("tracking_marker", "tracking_origin", rospy.Time().now(), rospy.Duration(10))
        # opt gives marker pose 
        X = np.array([opt.transform.translation.x, opt.transform.translation.y, opt.transform.translation.z])
        # camera_info = rospy.wait_for_message("/rs_l515/color/camera_info", CameraInfo)
        # K = np.array(camera_info.K).reshape(3, 3)
        K = np.array([[50, 0, 320], [0, 50, 240], [0, 0, 1]])
        # While not found N images
        final_poses = []
        num_images = 0
        while num_images < 10:
            #  Sample random relative pose
            R = rotation_matrix(np.random.uniform(-angle_delta, angle_delta), [1, 0, 0]).dot(
                rotation_matrix(np.random.uniform(-angle_delta, angle_delta), [0, 1, 0])).dot(
                rotation_matrix(np.random.uniform(-angle_delta, angle_delta), [0, 0, 1]))
            t = np.array([np.random.uniform(-translation_delta, translation_delta),
                          np.random.uniform(-translation_delta, translation_delta),
                          np.random.uniform(-translation_delta, translation_delta)])
            #  Apply relative pose to marker pose
            X_t = R[:3, :3].dot(X) + t
            #  Check if pose projects into image
            x = K.dot(X_t)
            x = (x / x[2])[:2]
            #  Accept if yes, otherwise reject and continue
            if x[0] >= 0 and x[0] < 640 and x[1] >= 0 and x[1] < 480:
                fp = deepcopy(start_pose)
                fp.pose.position = Point(*(R[:3, :3].dot(np.array([fp.pose.position.x, fp.pose.position.y, fp.pose.position.z])) + t))
                fp.pose.orientation = Quaternion(*quaternion_multiply(quaternion_from_matrix(R), np.array([fp.pose.orientation.x,
                                                                                                           fp.pose.orientation.y,
                                                                                                           fp.pose.orientation.z,
                                                                                                           fp.pose.orientation.w])))
                # Check if we can plan to the pose
                if self._plan_to_pose(fp):
                    
                    final_poses.append(fp)
                    num_images += 1

        # basis = np.eye(3)

        # pos_deltas = [quaternion_from_euler(*rot_axis * angle_delta) for rot_axis in basis]
        # neg_deltas = [quaternion_from_euler(*rot_axis * (-angle_delta)) for rot_axis in basis]

        # quaternion_deltas = list(chain.from_iterable(zip(pos_deltas, neg_deltas)))  # interleave

        # final_rots = []
        # for qd in quaternion_deltas:
        #     final_rots.append(list(qd))

        # # TODO: accept a list of delta values

        # pos_deltas = [quaternion_from_euler(*rot_axis * angle_delta / 2) for rot_axis in basis]
        # neg_deltas = [quaternion_from_euler(*rot_axis * (-angle_delta / 2)) for rot_axis in basis]

        # quaternion_deltas = list(chain.from_iterable(zip(pos_deltas, neg_deltas)))  # interleave
        # for qd in quaternion_deltas:
        #     final_rots.append(list(qd))

        # final_poses = []
        # for rot in final_rots:
        #     fp = deepcopy(start_pose)
        #     ori = fp.pose.orientation
        #     combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
        #     fp.pose.orientation = Quaternion(*combined_rot)
        #     final_poses.append(fp)

        # fp = deepcopy(start_pose)
        # fp.pose.position.x += translation_delta / 2
        # final_poses.append(fp)

        # fp = deepcopy(start_pose)
        # fp.pose.position.x -= translation_delta / 2
        # final_poses.append(fp)

        # fp = deepcopy(start_pose)
        # fp.pose.position.y += translation_delta
        # final_poses.append(fp)

        # fp = deepcopy(start_pose)
        # fp.pose.position.y -= translation_delta
        # final_poses.append(fp)

        # fp = deepcopy(start_pose)
        # fp.pose.position.z += translation_delta / 3
        # final_poses.append(fp)

        return final_poses

    @staticmethod
    def _rot_per_joint(plan, degrees=False):
        np_traj = np.array([p.positions for p in plan.joint_trajectory.points])
        if len(np_traj) == 0:
            raise ValueError
        np_traj_max_per_joint = np_traj.max(axis=0)
        np_traj_min_per_joint = np_traj.min(axis=0)
        ret = abs(np_traj_max_per_joint - np_traj_min_per_joint)
        if degrees:
            ret = [math.degrees(j) for j in ret]
        return ret

    @staticmethod
    def _is_crazy_plan(plan, max_rotation_per_joint):
        if len(plan.joint_trajectory.points) == 0:
            return False
        
        abs_rot_per_joint = CalibrationMovements._rot_per_joint(plan)
        if (abs_rot_per_joint > max_rotation_per_joint).any():
            return True
        else:
            return False
