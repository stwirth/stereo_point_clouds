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
import os
import sys
import argparse
import sensor_msgs.msg
import sensor_msgs.srv
import camera_info_parser
import argparse

def save_camera_info(req):
  filename = rospy.get_param("~filename")
  try:
    camera_info_parser.save_yaml(req.camera_info)
    return true, "success"
  except:
    return false, "saving failed"

if __name__ == "__main__":
  rospy.init_node('camera_info_saver')
  try:
    rospy.get_param("~filename")
    s = rospy.Service('set_camera_info', sensor_msgs.srv.SetCameraInfo, save_camera_info)
    rospy.spin()
  except KeyError as e:
    print 'Required parameter missing:', e
  except Exception, e:
    import traceback
    traceback.print_exc()
