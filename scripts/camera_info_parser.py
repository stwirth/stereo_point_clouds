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
import yaml
import sensor_msgs.msg

def parse_yaml(filename):
  stream = file(filename, 'r')
  calib_data = yaml.load(stream)
  cam_info = sensor_msgs.msg.CameraInfo()
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']
  cam_info.K = calib_data['camera_matrix']['data']
  cam_info.D = calib_data['distortion_coefficients']['data']
  cam_info.R = calib_data['rectification_matrix']['data']
  cam_info.P = calib_data['projection_matrix']['data']
  cam_info.distortion_model = calib_data['distortion_model']
  return cam_info

def save_yaml(cam_info, filename):
  calib_data = {}
  calib_data['image_width'] = cam_info.width 
  calib_data['image_height'] = cam_info.height
  calib_data['camera_matrix'] = {}
  calib_data['camera_matrix']['rows'] = 3
  calib_data['camera_matrix']['cols'] = 3
  calib_data['camera_matrix']['data'] = cam_info.K
  calib_data['distortion_coefficients'] = {}
  calib_data['distortion_coefficients']['rows'] = 1
  calib_data['distortion_coefficients']['cols'] = len(cam_info.D)
  calib_data['distortion_coefficients']['data'] = cam_info.D 
  calib_data['rectification_matrix'] = {}
  calib_data['rectification_matrix']['rows'] = 3
  calib_data['rectification_matrix']['cols'] = 3
  calib_data['rectification_matrix']['data'] = cam_info.R
  calib_data['projection_matrix'] = {}
  calib_data['projection_matrix']['rows'] = 3 
  calib_data['projection_matrix']['cols'] = 4 
  calib_data['projection_matrix']['data'] = cam_info.P 
  calib_data['distortion_model'] = cam_info.distortion_model
  stream = file(filename, 's')
  yaml.dump(calib_data, stream)


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='Parses camera info yaml files and returns them as sensor_msgs.msg.CameraInfo.')
  parser.add_argument('filename', help='input yaml file')
  args = parser.parse_args()
  try:
    info = parse_yaml(args.filename)
    print 'Read the following info from', args.filename, '\n', info
  except Exception, e:
    import traceback
    traceback.print_exc()
