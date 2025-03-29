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

#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>

namespace rclcpp
{
class Logger;
}

class DepthRegistration
{
public:
  enum Method
  {
    DEFAULT = 0,
    CPU,
    OPENCL
  };

protected:
  cv::Mat cameraMatrixRegistered, cameraMatrixDepth, rotation, translation, mapX, mapY;
  cv::Size sizeRegistered, sizeDepth;
  float zNear, zFar;

  DepthRegistration();

  virtual bool init(rclcpp::Logger& logger, const int deviceId) = 0;

public:
  virtual ~DepthRegistration();

  bool init(rclcpp::Logger& logger,
            const cv::Mat &cameraMatrixRegistered, const cv::Size &sizeRegistered, const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth,
            const cv::Mat &distortionDepth, const cv::Mat &rotation, const cv::Mat &translation,
            const float zNear = 0.5f, const float zFar = 12.0f, const int deviceId = -1);

  virtual bool registerDepth(rclcpp::Logger& logger, const cv::Mat &depth, cv::Mat &registered) = 0;

  static DepthRegistration *New(rclcpp::Logger& logger, Method method = DEFAULT);
};
