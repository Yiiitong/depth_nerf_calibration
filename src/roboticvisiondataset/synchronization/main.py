# coding=utf-8
import rospy
import message_filters
import sensor_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PIL_Image
import cv2


class RetrieveData:
    def __init__(self):
        self.bridge = CvBridge()

    def callback(self, stereo_left_rgb, rs_rgb, rs_depth):
        cv_stereo_left = self.bridge.imgmsg_to_cv2(stereo_left_rgb, desired_encoding='passthrough')
        cv_rs_rgb = self.bridge.imgmsg_to_cv2(rs_rgb, desired_encoding='passthrough')
        # cv_rs_depth = self.bridge.imgmsg_to_cv2(rs_depth, desired_encoding='passthrough')
        im1 = PIL_Image.fromarray(cv_stereo_left)
        im2 = PIL_Image.fromarray(cv_rs_rgb)
        # im3 = PIL_Image.fromarray(cv_rs_depth)
        im1.save("Images/stereo_left_raw/{}.jpeg".format(stereo_left_rgb.header.stamp.secs))
        im2.save("Images/rs_rgb_img/{}.jpeg".format(rs_rgb.header.stamp.secs))
        # im3.save("Images/rs_depth_img/{}.jpeg".format(rs_depth.header.stamp.secs))

    def data_pipe(self):
        rospy.init_node('img_listener')
        stereo_left_img_sub = message_filters.Subscriber('/arena_camera_node/image_raw', Image)
        rs_rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        rs_depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([stereo_left_img_sub, rs_rgb_sub, rs_depth_sub],
                                                         10, 0.01, allow_headerless=False)
        ts.registerCallback(self.callback)
        rospy.spin()


def main():
    retrieve_data = RetrieveData()
    retrieve_data.data_pipe()


if __name__ == '__main__':
    main()
