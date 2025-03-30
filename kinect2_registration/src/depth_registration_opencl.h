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

#include <kinect2_registration/kinect2_registration.h>

class DepthRegistrationOpenCL : public DepthRegistration
{
private:
  struct OCLData;

  OCLData *data;

public:
  DepthRegistrationOpenCL();

  ~DepthRegistrationOpenCL();

  bool init(const int deviceId);

  bool registerDepth(const cv::Mat &depth, cv::Mat &registered);

private:
  void generateOptions(std::string &options) const;

  bool readProgram(std::string &source) const;
};
