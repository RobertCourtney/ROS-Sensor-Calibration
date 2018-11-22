#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from visualization_msgs.msg import Marker
import tf
import roslib

marker = Marker()
marker.header.frame_id = "/car"
marker.header.stamp = rospy.Time()
marker.ns = "primitive_surfaces"
marker.id = 1
marker.type = marker.MESH_RESOURCE
marker.action = marker.ADD
marker.pose.position.x = 0
marker.pose.position.y = 0.85
marker.pose.position.z = 0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 1.0
marker.pose.orientation.w = 0.0
marker.scale.x = 0.141176470
marker.scale.y = 0.141176470
marker.scale.z = 0.141176470

marker.color.a = 0.0
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.mesh_use_embedded_materials  = True
marker.mesh_resource = "package://ros_camera_lidar_calib/data/jlr.dae";
        
def marker_pub():
    pub_m   = rospy.Publisher('surface', Marker, queue_size=100)
    
    rospy.init_node('jlr_car', anonymous=True)
    #br = tf.TransformBroadcaster()
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        #br.sendTransform((0.0,0.0,0.0), (0.0,0.0,0.0,1.0), rospy.Time.now(),"map", "surface_tf")
        marker.header.stamp = rospy.Time()
        pub_m.publish(marker)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        marker_pub()
    except rospy.ROSInterruptException:
        pass
        