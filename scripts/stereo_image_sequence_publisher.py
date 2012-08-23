#!/usr/bin/python
"""
Copyright (c) 2012,
Stephan Wirth (stephan.wirth@gmail.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of 
      the Balearican Islands nor the names of its contributors may be used to 
      endorse or promote products derived from this software without specific 
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


PKG = 'stereo_point_clouds' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse
import sensor_msgs.msg
import cv_bridge
import camera_info_parser
import glob
import cv
import argparse

def collect_image_files(image_dir, left_image_pattern, right_image_pattern):
  left_jpg_images = glob.glob(image_dir + '/' + left_image_pattern)
  right_jpg_images = glob.glob(image_dir + '/' + right_image_pattern)
  left_jpg_images.sort()
  right_jpg_images.sort()
  assert(len(left_jpg_images) == len(right_jpg_images));
  return zip(left_jpg_images, right_jpg_images)

def playback_images(image_dir, left_image_pattern, right_image_pattern, left_camera_info_file,right_camera_info_file,publish_rate):
  try:
    left_cam_info = camera_info_parser.parse_yaml(left_camera_info_file)
    right_cam_info = camera_info_parser.parse_yaml(right_camera_info_file)
    left_cam_info_publisher = rospy.Publisher('camera/left/camera_info', sensor_msgs.msg.CameraInfo)
    right_cam_info_publisher = rospy.Publisher('camera/right/camera_info', sensor_msgs.msg.CameraInfo)
  except IOError:
    print 'Camera info could not be loaded, will not be published.'
  image_files = collect_image_files(image_dir, left_image_pattern, right_image_pattern)
  rospy.loginfo('Found %i images in %s.',len(image_files),image_dir)
  bridge = cv_bridge.CvBridge()
  rate = rospy.Rate(publish_rate)
  left_image_publisher = rospy.Publisher('camera/left/image', sensor_msgs.msg.Image)
  right_image_publisher = rospy.Publisher('camera/right/image', sensor_msgs.msg.Image)
  left_cam_info_publisher = rospy.Publisher('camera/left/camera_info', sensor_msgs.msg.CameraInfo)
  right_cam_info_publisher = rospy.Publisher('camera/right/camera_info', sensor_msgs.msg.CameraInfo)
  rospy.loginfo('Starting playback.')
  for left_image_file, right_image_file in image_files:
    if rospy.is_shutdown():
      break
    now = rospy.Time.now()
    left_image = cv.LoadImage(left_image_file)
    right_image = cv.LoadImage(right_image_file)
    left_image_msg = bridge.cv_to_imgmsg(left_image, encoding='rgb8')
    right_image_msg = bridge.cv_to_imgmsg(right_image, encoding='rgb8')
    left_image_msg.header.stamp = now
    right_image_msg.header.stamp = now
    left_image_msg.header.frame_id = "/camera"
    right_image_msg.header.frame_id = "/camera"
    left_image_publisher.publish(left_image_msg)
    right_image_publisher.publish(right_image_msg)
    try:
      left_cam_info.header.stamp = now
      right_cam_info.header.stamp = now
      left_cam_info.header.frame_id = "/camera"
      right_cam_info.header.frame_id = "/camera"
      left_cam_info_publisher.publish(cam_info)
      right_cam_info_publisher.publish(cam_info)
    except NameError:
      pass
    rate.sleep()
  rospy.loginfo('No more images left. Stopping.')

if __name__ == "__main__":
  rospy.init_node('stereo_image_sequence_publisher')
  try:
    image_dir = rospy.get_param("~image_dir")
    left_camera_info_file = rospy.get_param("~left_camera_info_file")
    right_camera_info_file = rospy.get_param("~right_camera_info_file")
    left_image_pattern = rospy.get_param("~left_image_pattern")
    right_image_pattern = rospy.get_param("~right_image_pattern")
    frequency = rospy.get_param("~frequency", 10)
    playback_images(image_dir, left_image_pattern, right_image_pattern, left_camera_info_file, right_camera_info_file, frequency)
  except KeyError as e:
    print 'Required parameter missing:', e
  except Exception, e:
    import traceback
    traceback.print_exc()
