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

#define CALIB_FILE_EXT      ".png"
#define CALIB_FILE_COLOR    "_color" CALIB_FILE_EXT
#define CALIB_FILE_IR       "_ir" CALIB_FILE_EXT
#define CALIB_FILE_IR_GREY  "_grey_ir" CALIB_FILE_EXT
#define CALIB_FILE_DEPTH    "_depth" CALIB_FILE_EXT

#define CALIB_POINTS_COLOR  "_color_points.yaml"
#define CALIB_POINTS_IR     "_ir_points.yaml"

#define CALIB_SYNC          "_sync"
#define CALIB_SYNC_COLOR    CALIB_SYNC CALIB_FILE_COLOR
#define CALIB_SYNC_IR       CALIB_SYNC CALIB_FILE_IR
#define CALIB_SYNC_IR_GREY  CALIB_SYNC CALIB_FILE_IR_GREY
