#!/usr/bin/env python


import rospy

from visualization_msgs.msg import MarkerArray, Marker
from body_pose_msgs.msg import SkeletonArrayMsg, SkeletonMsg

def callback(data):
  skeleton = []
  markerArray = MarkerArray()
  for i,skel in enumerate(data.skeletons):
    for k, joint in enumerate(skel.joints):
      marker = Marker()
      marker.id = k + i * len(skel.joints)
      marker.header.frame_id = "world"
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2
      marker.color.a = 1.0
      marker.color.b = b
      marker.color.g = g
      marker.color.r = r
      marker.pose.orientation.w = 1.0
      marker.pose.position.x = joint.x
      marker.pose.position.y = joint.y
      marker.pose.position.z = joint.z
      markerArray.markers.append(marker)
  publisher.publish(markerArray)

rospy.init_node('skeleton_visualization')

skeleton_topic_to_read = rospy.get_param("~skeleton_topic_to_visualize", "/skeletons/skeleton")
topic_to_publish = rospy.get_param("~topic_to_publish",'visualization_marker_array')
r = rospy.get_param("~red", 1.0)
b = rospy.get_param("~blue", 0.0)
g = rospy.get_param("~green", 0.0)
publisher = rospy.Publisher(topic_to_publish, MarkerArray, queue_size=5)
rospy.Subscriber(skeleton_topic_to_read, SkeletonArrayMsg, callback, queue_size=1)

while not rospy.is_shutdown():
  rospy.sleep(0.01)
import sys
sys.exit(1)