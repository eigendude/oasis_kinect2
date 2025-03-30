/*
 * Copyright (C) 2025 Garrett Brown
 * This file is part of OASIS Kinect2 - https://github.com/eigendude/oasis_kinect2
 *
 * Copyright (C) 2014 University of Bremen, Institute for Artificial Intelligence
 * This file is derived from IAI Kinect2 - https://github.com/code-iai/iai_kinect2
 *
 * SPDX-License-Identifier: Apache-2.0
 * See the file LICENSE for more information.
 */
#pragma once

#ifndef __KINECT2_DEFINITIONS_H__
#define __KINECT2_DEFINITIONS_H__

#include <kinect2_registration/kinect2_console.h>

#define K2_DEFAULT_NS          "kinect2"

#define K2_TF_LINK             "_link"
#define K2_TF_RGB_OPT_FRAME    "_rgb_optical_frame"
#define K2_TF_IR_OPT_FRAME     "_ir_optical_frame"

#define K2_TOPIC_HD            "/hd"
#define K2_TOPIC_QHD           "/qhd"
#define K2_TOPIC_SD            "/sd"

#define K2_TOPIC_IMAGE_RECT    "_rect"
#define K2_TOPIC_IMAGE_COLOR   "/image_color"
#define K2_TOPIC_IMAGE_MONO    "/image_mono"
#define K2_TOPIC_IMAGE_DEPTH   "/image_depth"
#define K2_TOPIC_IMAGE_IR      "/image_ir"

#define K2_TOPIC_COMPRESSED    "/compressed"
#define K2_TOPIC_INFO          "/camera_info"

#define K2_CALIB_COLOR         "calib_color.yaml"
#define K2_CALIB_IR            "calib_ir.yaml"
#define K2_CALIB_POSE          "calib_pose.yaml"
#define K2_CALIB_DEPTH         "calib_depth.yaml"

#define K2_CALIB_CAMERA_MATRIX "cameraMatrix"
#define K2_CALIB_DISTORTION    "distortionCoefficients"
#define K2_CALIB_ROTATION      "rotation"
#define K2_CALIB_PROJECTION    "projection"
#define K2_CALIB_TRANSLATION   "translation"
#define K2_CALIB_ESSENTIAL     "essential"
#define K2_CALIB_FUNDAMENTAL   "fundamental"
#define K2_CALIB_DEPTH_SHIFT   "depthShift"

#endif //__KINECT2_DEFINITIONS_H__
