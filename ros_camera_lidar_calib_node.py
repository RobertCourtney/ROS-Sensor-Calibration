#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Import ROS-Tensorflow interface
import rospy
import message_filters
from std_msgs.msg import String
import roslib
import tf

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, ChannelFloat32
from std_msgs.msg import Int16
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import rosbag
import cv2
#import pcl
import yaml
import numpy as np

from multiprocessing import Queue, Pool
from cv_bridge import CvBridge, CvBridgeError


_cv_bridge = CvBridge()
_laser_projector = LaserProjection()


# Camera rectification?? To be improved
# Put here the calibration of the camera
DIM=(1920, 1208)
K=np.array([[693.506921, 0.000000, 955.729444], [0.000000, 694.129349, 641.732500], [0.0, 0.0, 1.0]])
D=np.array([[-0.022636], [ -0.033855], [0.013493], [-0.001831]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)


def cam0_img_compre_callback(image_msg):
    # Get image as np
    image_np = _cv_bridge.compressed_imgmsg_to_cv2(image_msg)
    undistorted_img = cv2.remap(image_np, map1, map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
    # print("cam_0 shape = ",image_np.shape)
    cv2.imshow("distorted0",image_np)
    cv2.imshow("undistorted0",undistorted_img)
    cv2.waitKey(10)

def cam1_img_compre_callback(image_msg):
    # Get image as np
    image_np = _cv_bridge.compressed_imgmsg_to_cv2(image_msg)
    undistorted_img = cv2.remap(image_np, map1, map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
    # print("cam_1 shape = ",image_np.shape)
    # cv2.imshow("distorted1",image_np)
    # cv2.waitKey(10)

def cam2_img_compre_callback(image_msg):
    # Get image as np
    image_np = _cv_bridge.compressed_imgmsg_to_cv2(image_msg)
    undistorted_img = cv2.remap(image_np, map1, map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
    # print("cam_2 shape = ",image_np.shape)
    # cv2.imshow("distorted2",image_np)
    # cv2.waitKey(10)

def cam3_img_compre_callback(image_msg):
    # Get image as np
    image_np = _cv_bridge.compressed_imgmsg_to_cv2(image_msg)
    undistorted_img = cv2.remap(image_np, map1, map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
    # print("cam_3 shape = ",image_np.shape)
    # cv2.imshow("distorted3",image_np)
    # cv2.waitKey(10)

def lidar_callback( scan):
    # rospy.loginfo("Got scan, projecting")
    #ros_cloud = laser_projector.projectLaser(scan)
    #pcl_cloud = ros_to_pcl(ros_cloud)
    # Magic here
    print("yes")

def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list = []
    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])
    #pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data



def camera_lidar_calib():
    rospy.init_node('ros_camera_lidar_calib', anonymous=True)
    listener = tf.TransformListener()

    cam0_subs_topic = '/gmsl_camera/port_0/cam_0/image_raw/compressed'
    cam1_subs_topic = '/gmsl_camera/port_0/cam_1/image_raw/compressed'
    cam2_subs_topic = '/gmsl_camera/port_0/cam_2/image_raw/compressed'
    #cam3_subs_topic = '/gmsl_camera/port_0/cam_3/image_raw/compressed'
    lidar_subs_topic = '/Sensor/points'


    cam0_img_sub = rospy.Subscriber( cam0_subs_topic , CompressedImage, cam0_img_compre_callback, queue_size=1)
    cam1_img_sub = rospy.Subscriber( cam1_subs_topic , CompressedImage, cam1_img_compre_callback, queue_size=1)
    cam2_img_sub = rospy.Subscriber( cam2_subs_topic , CompressedImage, cam2_img_compre_callback, queue_size=1)
    #cam3_img_sub = rospy.Subscriber( cam3_subs_topic , CompressedImage, cam3_img_compre_callback, queue_size=1)
    lidar_sub = rospy.Subscriber( lidar_subs_topic , PointCloud2, lidar_callback, queue_size=2)
    # #
    # now = rospy.Time.now()
    # (trans,rot) = listener.lookupTransform("/port0_cam0","/Sensor", now)
    # print(trans,rot)
    # ts = message_filters.TimeSynchronizer([cam0_img_sub, cam1_img_sub, cam2_img_sub], 10)
    # ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        camera_lidar_calib()
    except rospy.ROSInterruptException:
        pass
