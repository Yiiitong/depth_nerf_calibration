#!/usr/bin/env python

# for small matrices, openblas spawns thread that eat the whole CPU for no reason
# see https://www.bountysource.com/issues/38121131-high-cpu-usage-by-kernel-while-using-inv-and-solve-from-linalg
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'

from ndi_polaris import tracking_probe
import tf
from tf import transformations as tfs
import rospy
import numpy as np
from os import path
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Header


def target_to_transformation(vec):
    trasf_list = list(vec.mat())
    return np.array(trasf_list).reshape(4, 4)


rospy.init_node('ndi_polaris')

object_dir = rospy.get_param('~ndi_tool_dir')
object_names_string = rospy.get_param('~ndi_tool_names')
object_names = object_names_string.split(',')
if not object_names:
    rospy.logfatal('At least the path to a tool ROM must be specified')
    raise ValueError('No NDI tools specified')
object_files = [path.join(object_dir, on + '.rom') for on in object_names]
rospy.loginfo('Using tools: ' + '\n'.join(object_files))

ndi_origin_name = rospy.get_param('~ndi_origin_name', 'optical_origin')
frequency = rospy.get_param('~ndi_publishing_frequency', 60)
devname = rospy.get_param('~ndi_device_name', '/dev/ttyUSB0')
baud_rate = rospy.get_param('~ndi_baud_rate', 115200)
from_origin_to_target = rospy.get_param('~from_origin_to_target', 'true')

if str(from_origin_to_target).lower() in ('yes', 'true', 't', '1'):
    from_origin_to_target = True
elif str(from_origin_to_target).lower() in ('no', 'false', 'f', '0'):
    if len(object_names) > 1:
        raise ValueError('It is impossible to broadcast the inverse transformations with more targets: '
                         'It would lead to an invalid tf tree')
    from_origin_to_target = False
else:
    raise ValueError('Invalid value for parameter from_origin_to_target: ' + str(from_origin_to_target))

broadcaster = tf.TransformBroadcaster()

publisher = rospy.Publisher('/ndi_transforms', TransformStamped, queue_size=10)

track = tracking_probe.NDIComm()
track.open(devname, int(baud_rate))
rospy.sleep(rospy.Duration(3))
for n, o in zip(object_names, object_files):
    track.registerTarget(n, o)
track.initDevice()
rospy.sleep(rospy.Duration(3))
track.startTracking()
rospy.sleep(rospy.Duration(3))
tframe = tracking_probe.TrackerFrame()

rate = rospy.Rate(int(frequency))
pose = TransformStamped(header=Header(frame_id=ndi_origin_name))

while not rospy.is_shutdown():
    track.grabFrame(tframe)
    frame_time = rospy.Time.now()
    for target in tframe.targets():
        if target.status() == 0:
            transf_matrix = target_to_transformation(target)
            if from_origin_to_target:
                transl = tfs.translation_from_matrix(transf_matrix)
                rot = tfs.quaternion_from_matrix(transf_matrix)
                rot = rot / np.linalg.norm(rot)
                broadcaster.sendTransform(transl,
                                          rot,
                                          frame_time,
                                          object_names[target.id() - 1],
                                          ndi_origin_name)
                pose.header.stamp = frame_time
                pose.header.frame_id = ndi_origin_name
                pose.child_frame_id = object_names[target.id() - 1]
                pose.transform.translation = Vector3(*transl)
                pose.transform.rotation = Quaternion(*rot)
                publisher.publish(pose)
            else:
                transf_matrix = np.linalg.inv(transf_matrix)
                transl = tfs.translation_from_matrix(transf_matrix)
                rot = tfs.quaternion_from_matrix(transf_matrix)
                rot = rot / np.linalg.norm(rot)
                broadcaster.sendTransform(transl,
                                          rot,
                                          frame_time,
                                          ndi_origin_name,
                                          object_names[target.id() - 1]
                )
                pose.header.stamp = frame_time
                pose.header.frame_id = object_names[target.id() - 1]
                pose.child_frame_id = ndi_origin_name
                pose.transform.translation = Vector3(*transl)
                pose.transform.rotation = Quaternion(*rot)
                publisher.publish(pose)
        else:
            rospy.logwarn('Not publishing frame for target %i, status: %i' % (target.id(), target.status()))
    rate.sleep()
