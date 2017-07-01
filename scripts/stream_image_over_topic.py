#!/usr/bin/env python

directory = "/home/marco/Desktop/BPR_dataset/camera0"
# camera_info_filename = "/home/marco/Desktop/BPR_dataset/camera0/calib_ir.yaml"
rgb_topic_to_stream = "/image/rgb"
depth_topic_to_stream = "/image/depth"
camera_info_topic_to_stream = "/depth/camera_info"
names_topic_to_stream = "/names"

import rospy
from sensor_msgs.msg import Image, CameraInfo

import std_msgs.msg

import cv2
from cv_bridge import CvBridge, CvBridgeError

import os
import fnmatch
import sys
import math
import numpy

def recursive_glob(rootdir='.', pattern='*', exclude=[]):
  """Search recursively for files matching a specified pattern.
  Adapted from http://stackoverflow.com/questions/2186525/use-a-glob-to-find-files-recursively-in-python
  """
  matches = []
  for root, dirnames, filenames in os.walk(rootdir):
    for filename in fnmatch.filter(filenames, pattern):
      add = True
      for exc in exclude:
        if exc in filename: 
          add = False
          break
      if add:
        matches.append(os.path.join(root, filename))
  return matches

if __name__ == "__main__":
  rospy.init_node('RGBDepthCameraInfoStreamer', anonymous=True)

  image_list_rgb = recursive_glob(directory, "*_rgb.png", [])
  image_list_depth = recursive_glob(directory, "*_depth.png", [])
  image_list_rgb.sort()
  image_list_depth.sort()

  rgb_pub = rospy.Publisher(rgb_topic_to_stream, Image, queue_size=10)
  depth_pub = rospy.Publisher(depth_topic_to_stream, Image, queue_size=10)
  camera_info_pub = rospy.Publisher(camera_info_topic_to_stream, CameraInfo, queue_size=10)
  names_pub = rospy.Publisher(names_topic_to_stream, std_msgs.msg.String, queue_size=10)
  yaml_data = numpy.asarray(cv2.cv.Load(directory + "/calib_ir.yaml"))
  
  cam_info = CameraInfo()
  cam_info.height = 540
  cam_info.width = 960
  cam_info.distortion_model = 'plumb_bob'
  cam_info.K[0] = yaml_data[0,0]
  cam_info.K[2] = yaml_data[0,2]
  cam_info.K[4] = yaml_data[1,1]
  cam_info.K[5] = yaml_data[1,2]
  cam_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
  cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  cam_info.P[0 * 4 + 0] = yaml_data[0,0]
  cam_info.P[0 * 4 + 2] = yaml_data[0,2]
  cam_info.P[1 * 4 + 1] = yaml_data[1,1]
  cam_info.P[1 * 4 + 2] = yaml_data[1,2]
  cam_info.P[2 * 4 + 2] = 1
  for i in xrange(len(image_list_rgb)):
    print (str(i).zfill(4) + " / " + str(len(image_list_rgb)))
    now = rospy.get_rostime()
    rgb_frame = cv2.imread(image_list_rgb[i], cv2.IMREAD_COLOR)
    depth_frame = cv2.imread(image_list_depth[i], cv2.IMREAD_GRAYSCALE)
    msg_frame_rgb = CvBridge().cv2_to_imgmsg(rgb_frame, encoding="bgr8")
    msg_frame_depth = CvBridge().cv2_to_imgmsg(depth_frame, encoding="mono8")
    msg_frame_rgb.header.stamp = now
    msg_frame_depth.header.stamp = now
    cam_info.header.stamp = now
    while (rospy.get_rostime() - now).to_sec() < 0.5:
      rgb_pub.publish(msg_frame_rgb)
      depth_pub.publish(msg_frame_depth)
      camera_info_pub.publish(cam_info)
      prefix = "/".join(image_list_depth[i].split("/")[:-1])
      name = image_list_depth[i].split("/")[-1][:-4]
      names_pub.publish(prefix + "/" + name)
      if rospy.is_shutdown():
        sys.exit(1)
      rospy.sleep(0.01)