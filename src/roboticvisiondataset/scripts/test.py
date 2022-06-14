#!/usr/bin/env python

from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point

import rospy, tf, math

rospy.init_node('capture_robotpose', anonymous=True)

br = tf.TransformBroadcaster()
rate = rospy.Rate(10.0)

while not rospy.is_shutdown():

    goal = PoseStamped()
    t = rospy.Time.now().to_sec() * math.pi
    br.sendTransform((2.0 * math.sin(t), 2.0 * math.cos(t), 0.0),(0.0,0.0,0.0,1.0),rospy.Time.now(),"center","robot_pose")
    print("broadcast")
    rate.sleep()   

    '''
    goal.header.stamp = rospy.Time(rospy.get_time())
    goal.header.frame_id = "robot_pose"

    goal.pose.position.x = 0
    goal.pose.position.y = 0
    goal.pose.position.z = 0

    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 1

    pose_publisher.publish(goal)
    '''

