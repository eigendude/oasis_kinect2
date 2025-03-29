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

#include <Eigen/Geometry>

#include <kinect2_registration/kinect2_registration.h>

class DepthRegistrationCPU : public DepthRegistration
{
private:
  cv::Mat lookupX, lookupY;
  Eigen::Matrix4d proj;
  double fx, fy, cx, cy;

public:
  DepthRegistrationCPU();

  ~DepthRegistrationCPU();

  bool init(rclcpp::Logger& logger, const int deviceId);

  bool registerDepth(rclcpp::Logger& logger, const cv::Mat &depth, cv::Mat &registered);

private:
  void createLookup();

  uint16_t interpolate(const cv::Mat &in, const float &x, const float &y) const;

  void remapDepth(const cv::Mat &depth, cv::Mat &scaled) const;
  void projectDepth(const cv::Mat &scaled, cv::Mat &registered) const;
};
