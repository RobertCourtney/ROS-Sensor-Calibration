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
import threading
import thread

from multiprocessing import Queue, Pool
from cv_bridge import CvBridge, CvBridgeError

class camera_lidar_calib(object):
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._laser_projector = LaserProjection()
    
        # # Camera rectification?? To be improved: read from .yaml file
        # Put here the calibration of the camera
        self.DIM    = (1920, 1208)
        self.K      = np.array([[693.506921, 0.000000, 955.729444], [0.000000, 694.129349, 641.732500], [0.0, 0.0, 1.0]])
        self.D      = np.array([[-0.022636], [ -0.033855], [0.013493], [-0.001831]])
        self.map1, self.map2    = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, self.DIM, cv2.CV_16SC2) # np.eye(3) here is actually the rotation matrix
        
        # # Declare subscribers to get the latest data
        cam0_subs_topic = '/gmsl_camera/port_0/cam_0/image_raw/compressed'
        cam1_subs_topic = '/gmsl_camera/port_0/cam_1/image_raw/compressed'
        cam2_subs_topic = '/gmsl_camera/port_0/cam_2/image_raw/compressed'
        #cam3_subs_topic = '/gmsl_camera/port_0/cam_3/image_raw/compressed'
        lidar_subs_topic = '/Sensor/points'

        self.cam0_img_sub   = rospy.Subscriber( cam0_subs_topic , CompressedImage, self.cam0_img_compre_callback, queue_size=1)
        #self.cam1_img_sub   = rospy.Subscriber( cam1_subs_topic , CompressedImage, self.cam1_img_compre_callback, queue_size=1)
        #self.cam2_img_sub   = rospy.Subscriber( cam2_subs_topic , CompressedImage, self.cam2_img_compre_callback, queue_size=1)
        #self.cam3_img_sub  = rospy.Subscriber( cam3_subs_topic , CompressedImage, self.cam3_img_compre_callback, queue_size=1)
        self.lidar_sub      = rospy.Subscriber( lidar_subs_topic , PointCloud2, self.ros_to_pcl, queue_size=2)
        # Get the tfs
        self.tf_listener = tf.TransformListener()
        
        # # Declare the global variables we will use to get the latest info
        self.cam0_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam0_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam1_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam1_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam2_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam2_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam3_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam3_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.pcl_cloud              = np.empty( (500,4) ) # Do not know the width of a normal scan. ight be variable too......
        
        # # Main loop: Data projections and alignment on real time
        thread.start_new_thread(self.projection_calibrate())
        
    def projection_calibrate(self ):
        # # Main loop: Data projections and alignment on real time
        rate = rospy.Rate(30.0) # ?
        rot_trans_matrix = np.empty(0)
        while not rospy.is_shutdown():
            # Get the tfs
            if not rot_trans_matrix.shape[0]: # Get the tf is empty
                try:
                    (trans,rot) = self.tf_listener.lookupTransform("/port0_cam0","/Sensor", rospy.Time(0)) # get different protections here, as needed
                    rot_trans_matrix = self.tf_listener.fromTranslationRotation(trans, rot)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            
            ## Projections
            #print(rot_trans_matrix[:3,:])
            #print( self.pcl_cloud.shape )
    
            cloud_pixels = np.dot(  self.K , np.dot(  rot_trans_matrix[:3,:] , self.pcl_cloud[:,:4].T  )  )[:2,:].astype(int)
            # filter pixels out of img
            cloud_pixels = cloud_pixels[ : , (cloud_pixels[0,:]>0) & (cloud_pixels[0,:]<self.DIM[1]) & (cloud_pixels[1,:]>0) & (cloud_pixels[1,:]<self.DIM[0]) ] # Filter those points outside the image

            self.cam0_undistorted_img[cloud_pixels[0,:],cloud_pixels[1,:],:] = [0,0,255] # plot onto img the points. We may have intensity or can apply a color map on distance
            
            # Show imgs
            cv2.imshow("undistorted0",self.cam0_undistorted_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # Loop!
            rate.sleep()
            
        cv2.destroyAllWindows()
        
    def cam0_img_compre_callback(self,image_msg):
        # Get image as np
        self.cam0_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
        self.cam0_undistorted_img = cv2.remap(self.cam0_image_np, self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
        # print("cam_0 shape = ",self.cam0_image_np.shape)
        ##cv2.imshow("distorted0",self.cam0_image_np)
        ##cv2.imshow("undistorted0",self.cam0_undistorted_img)
        ##cv2.waitKey(10)

    def cam1_img_compre_callback(self,image_msg):
        # Get image as np
        self.cam1_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
        self.cam1_undistorted_img = cv2.remap(self.cam0_image_np,  self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
        # print("cam_1 shape = ",self.cam0_image_np.shape)
        # cv2.imshow("distorted1",self.cam0_image_np)
        # cv2.waitKey(10)

    def cam2_img_compre_callback(self,image_msg):
        # Get image as np
        self.cam2_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
        self.cam2_undistorted_img = cv2.remap(self.cam0_image_np, self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
        # print("cam_2 shape = ",self.cam0_image_np.shape)
        # cv2.imshow("distorted2",self.cam0_image_np)
        # cv2.waitKey(10)

    def cam3_img_compre_callback(self,image_msg):
        # Get image as np
        self.cam3_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
        self.cam3_undistorted_img = cv2.remap(self.cam0_image_np,  self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
        # print("cam_3 shape = ",self.cam0_image_np.shape)
        # cv2.imshow("distorted3",self.cam0_image_np)
        # cv2.waitKey(10)

    def lidar_callback(self, scan): # scan msg has the following format http://docs.ros.org/jade/api/sensor_msgs/html/msg/PointCloud2.html
        rospy.loginfo("Got scan, projecting")
        #ros_cloud_msg = self._laser_projector.projectLaser(scan)
        # Lidar filtering?? In case of angular filtering?
        # ros_cloud_msg 
        #self.pcl_cloud = self.ros_to_pcl(ros_cloud_msg)
        
    def ros_to_pcl(self,ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message
            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2],1.0, data[3]])
        #pcl_data = pcl.PointCloud_PointXYZRGB()
        #pcl_data.from_list(points_list)
        
        self.pcl_cloud = np.array(points_list)

    def spin():
        rospy.spin()


