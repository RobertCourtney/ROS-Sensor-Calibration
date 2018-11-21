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
from image_geometry import PinholeCameraModel
from camera_info_manager import CameraInfoManager  ## Needs to install https://github.com/ros-perception/camera_info_manager_py

from std_msgs.msg import Int16
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from laser_geometry import LaserProjection
import rosbag
import cv2
import lidar_dump_numpy as pcl2_2_np

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
        # self.DIM    = (1920, 1208)
        # self.K      = np.array([[693.506921, 0.000000, 955.729444], [0.000000, 694.129349, 641.732500], [0.0, 0.0, 1.0]])
        # self.D      = np.array([[-0.022636], [ -0.033855], [0.013493], [-0.001831]])
        # self.map1, self.map2    = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, self.DIM, cv2.CV_16SC2) # np.eye(3) here is actually the rotation matrix

        #   OR load it from a yaml file
        self.cameraModel = PinholeCameraModel()

        # See https://github.com/ros-perception/camera_info_manager_py/tree/master/tests
        camera_infomanager = CameraInfoManager(cname='truefisheye',url='package://ros_camera_lidar_calib/cfg/truefisheye800x503.yaml') # Select the calibration file
        camera_infomanager.loadCameraInfo()
        self.cameraInfo = camera_infomanager.getCameraInfo()
        # Crete a camera from camera info
        self.cameraModel.fromCameraInfo( self.cameraInfo )# Create camera model
        self.DIM    = (self.cameraInfo.width,self.cameraInfo.height)
        # Get rectification maps
        self.map1, self.map2    = cv2.fisheye.initUndistortRectifyMap(self.cameraModel.intrinsicMatrix(), self.cameraModel.distortionCoeffs(),
                                np.eye(3), self.cameraModel.intrinsicMatrix(), (self.cameraInfo.width,self.cameraInfo.height), cv2.CV_16SC2) # np.eye(3) here is actually the rotation matrix

        # # Declare subscribers to get the latest data
        cam0_subs_topic = '/gmsl_camera/port_0/cam_0/image_raw'
        cam1_subs_topic = '/gmsl_camera/port_0/cam_1/image_raw'
        cam2_subs_topic = '/gmsl_camera/port_0/cam_2/image_raw'
        #cam3_subs_topic = '/gmsl_camera/port_0/cam_3/image_raw/compressed'
        lidar_subs_topic = '/Sensor/points'

        #self.cam0_img_sub   = rospy.Subscriber( cam0_subs_topic , Image, self.cam0_img_callback, queue_size=1)
        #self.cam1_img_sub   = rospy.Subscriber( cam1_subs_topic , Image, self.cam1_img_callback, queue_size=1)
        self.cam2_img_sub   = rospy.Subscriber( cam2_subs_topic , Image, self.cam2_img_callback, queue_size=1)
        #self.cam0_img_sub   = rospy.Subscriber( cam0_subs_topic , CompressedImage, self.cam0_img_compre_callback, queue_size=1)
        #self.cam1_img_sub   = rospy.Subscriber( cam1_subs_topic , CompressedImage, self.cam1_img_compre_callback, queue_size=1)
        #self.cam2_img_sub   = rospy.Subscriber( cam2_subs_topic , CompressedImage, self.cam2_img_compre_callback, queue_size=1)
        #self.cam3_img_sub  = rospy.Subscriber( cam3_subs_topic , CompressedImage, self.cam3_img_compre_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber( lidar_subs_topic , PointCloud2, self.lidar_callback, queue_size=1)
        # Get the tfs
        self.tf_listener = tf.TransformListener()
        #self.lidar_time = rospy.Subscriber(lidar_subs_topic , PointCloud2, self.readtimestamp)
        #self.img0_time = rospy.Subscriber(cam0_subs_topic , CompressedImage, self.readtimestamp)

        # # Declare the global variables we will use to get the latest info
        self.cam0_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam0_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam1_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam1_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam2_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam2_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam3_image_np          = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.cam3_undistorted_img   = np.empty( (self.DIM[1],self.DIM[0],3) )
        self.pcl_cloud              = np.empty( (500,4) ) # Do not know the width of a normal scan. might be variable too......

        self.now = rospy.Time.now()

        # # Main loop: Data projections and alignment on real time
        self.lidar_angle_range_interest = [0,180] # From -180 to 180. Front is 0deg. Put the range of angles we may want to get points from. Depending of camera etc
        thread.start_new_thread(self.projection_calibrate())

    def projection_calibrate(self ):
        # # Main loop: Data projections and alignment on real time
        rate = rospy.Rate(30) # ?
        rot_trans_matrix = np.empty(0)
        while not rospy.is_shutdown():
            # Get the tfs
            if not rot_trans_matrix.shape[0]: # Get the tf is empty
                try:
                    (trans,rot) = self.tf_listener.lookupTransform("/port0_cam3","/Sensor", rospy.Time(0)) # get different protections here, as needed
                    rot_trans_matrix = self.tf_listener.fromTranslationRotation(trans, rot)
                    # print(trans)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            ## Projections: see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
            # # project onto camera frame:
            cloud_oncam =  np.dot(  rot_trans_matrix[:3,:] , self.pcl_cloud[:,:4].T  )
            # Filter points that wont be on the image: remove points behind the camera plane
            #cloud_oncam = cloud_oncam[:,cloud_oncam[2,:]>0.01]
            cloud_oncam_2d = cloud_oncam/cloud_oncam[2,:] # project 3D to 2D plane

            # # Project onto images pixels
            #cloud_pixels =  np.dot( self.K , cloud_oncam_2d ).astype(int) # [u,v].T, u->x-.Horizontal, v->y->vertical
            cloud_pixels =  np.dot(  np.array(self.cameraModel.intrinsicMatrix()) , cloud_oncam_2d ).astype(int) # [u,v,1].T, u->x-.Horizontal, v->y->vertical
            cloud_pixels[2,:] = cloud_oncam[2,:] #Append on the last dim the real depth. Not in camera plane dimensions. DEPTH: [u,v,depth in m].T

            # filter pixels out of img
            padding = 2
            cloud_pixels = cloud_pixels[ : , (cloud_pixels[0,:]>padding) & (cloud_pixels[0,:]<(self.DIM[0]-padding)) & (cloud_pixels[1,:]>padding) & (cloud_pixels[1,:]<(self.DIM[1]-padding) ) ] # Filter those points outside the image

            image = self.cam2_undistorted_img # improve loop performance by putting it in a variable

            for idx in range(len(cloud_pixels[2])):
                image[cloud_pixels[1,idx],cloud_pixels[0,idx],:] = [0,0,np.clip(cloud_pixels[2,idx]*20,0,255)]

            self.cam2_undistorted_img = image

            # Show imgs
            cv2.imshow("undistorted2",self.cam2_undistorted_img)
            #cv2.imshow("distorted0",self.cam0_image_np)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # Loop!
            rate.sleep()

        cv2.destroyAllWindows()

    def readtimestamp(self, data):
        print(data.header.stamp)
        #print(data.header.frame_id,data.header.stamp,data.header.seq)

    def cam0_img_callback(self,image_msg):
        # Get image as np
        # self.cam0_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
        self.cam0_image_np = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.cam0_undistorted_img = cv2.remap(self.cam0_image_np, self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)

    def cam1_img_callback(self,image_msg):
        # Get image as np
        # self.cam0_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
        self.cam1_image_np = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.cam1_undistorted_img = cv2.remap(self.cam1_image_np, self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)

    def cam2_img_callback(self,image_msg):
        # Get image as np
        # self.cam0_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
        self.cam2_image_np = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.cam2_undistorted_img = cv2.remap(self.cam2_image_np, self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)

    # def cam0_img_compre_callback(self,image_msg):
    #     # Get image as np
    #     self.cam0_image_np = self._cv_bridge.compre_imgmsg_to_cv2(image_msg)
    #     #self.cam0_image_np = cv2.imdecode( np.fromstring(image_msg.data, np.uint8) , cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    #
    #     self.cam0_undistorted_img = cv2.remap(self.cam0_image_np, self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
    #
    # def cam1_img_compre_callback(self,image_msg):
    #     # Get image as np
    #     self.cam1_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
    #     self.cam1_undistorted_img = cv2.remap(self.cam0_image_np,  self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
    #
    # def cam2_img_compre_callback(self,image_msg):
    #     # Get image as np
    #     self.cam2_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
    #     self.cam2_undistorted_img = cv2.remap(self.cam0_image_np, self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
    #
    # def cam3_img_compre_callback(self,image_msg):
    #     # Get image as np
    #     self.cam3_image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg)
    #     self.cam3_undistorted_img = cv2.remap(self.cam0_image_np,  self.map1,  self.map2, interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)


    def lidar_callback(self,lidar_scan):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
            Args:
                lidar_scan (PointCloud2): ROS PointCloud2 message
            Returns:
                np pointcloud: numpy XYZ1I point cloud
        """
        # points_list = []
        # for data in pc2.read_points(lidar_scan, skip_nans=True):
            # points_list.append([data[0], data[1], data[2],1.0])
        # self.pcl_cloud = np.array(points_list)

        # 2nd method
        #self.pcl_cloud =  np.array( list(pc2.read_points(lidar_scan, skip_nans=True)) )

        # 3rd method
        nparr = pcl2_2_np.msg_to_arr(lidar_scan)
        width = nparr.shape[1]
        m = width/360
        self.pcl_cloud = self.lidar_map_flatten( nparr[:, int(width/2.0 + self.lidar_angle_range_interest[0]*m):int(width/2.0 + self.lidar_angle_range_interest[1]*m)]  ) # we cloud filter here by angle!

        # Debugging tools
        # print "  Got cloud @ ", 1.0/float(rospy.Time.now().to_sec() - self.now.to_sec()) ," Hz "
        # self.now = rospy.Time.now()

    def lidar_map_flatten(self, nparr):
        """ Converts a numpy array of (height, width) [x,y,z,i,_] into a flatten np array of [xyz1,numer_of_points]
            Args:
                nparr (numpy): numpy array of (8, ~5440) [x,y,z,i,_]loud @ ", 1.0/float(rospy.Time.now().to_sec() - self.now.to_sec()) ," Hz "
        self.now = ros
            Returns:
                np pointcloud: numpy [xyz1,numer_of_points]  point cloud
        """
        pcl_cloud   = np.ones( (nparr.shape[0]*nparr.shape[1],4) )
        pcl_cloud[:,0]  = np.reshape( nparr['x'], -1 )
        pcl_cloud[:,1]  = np.reshape( nparr['y'], -1 )
        pcl_cloud[:,2]  = np.reshape( nparr['z'], -1 )

        return pcl_cloud
    def spin(self):
        rospy.spin()
