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

#include <kinect2_registration/kinect2_registration.h>
#include <kinect2_registration/kinect2_console.h>

#ifdef DEPTH_REG_CPU
#include "depth_registration_cpu.h"
#endif

#ifdef DEPTH_REG_OPENCL
#include "depth_registration_opencl.h"
#endif

DepthRegistration::DepthRegistration()
{
}

DepthRegistration::~DepthRegistration()
{
}

bool DepthRegistration::init(const cv::Mat &cameraMatrixRegistered, const cv::Size &sizeRegistered, const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth,
                             const cv::Mat &distortionDepth, const cv::Mat &rotation, const cv::Mat &translation,
                             const float zNear, const float zFar, const int deviceId)
{
  this->cameraMatrixRegistered = cameraMatrixRegistered;
  this->cameraMatrixDepth = cameraMatrixDepth;
  this->rotation = rotation;
  this->translation = translation;
  this->sizeRegistered = sizeRegistered;
  this->sizeDepth = sizeDepth;
  this->zNear = zNear;
  this->zFar = zFar;

  cv::initUndistortRectifyMap(cameraMatrixDepth, distortionDepth, cv::Mat(), cameraMatrixRegistered, sizeRegistered, CV_32FC1, mapX, mapY);

  return init(deviceId);
}

DepthRegistration *DepthRegistration::New(rclcpp::Logger logger, Method method)
{
  if(method == DEFAULT)
  {
#ifdef DEPTH_REG_OPENCL
    method = OPENCL;
#elif defined DEPTH_REG_CPU
    method = CPU;
#endif
  }

  switch(method)
  {
  case DEFAULT:
    OUT_ERROR(logger, "No default registration method available!");
    break;
  case CPU:
#ifdef DEPTH_REG_CPU
    OUT_INFO(logger, "Using CPU registration method!");
    return new DepthRegistrationCPU();
#else
    OUT_ERROR(logger, "CPU registration method not available!");
    break;
#endif
  case OPENCL:
#ifdef DEPTH_REG_OPENCL
    OUT_INFO(logger, "Using OpenCL registration method!");
    return new DepthRegistrationOpenCL();
#else
    OUT_ERROR(logger, "OpenCL registration method not available!");
    break;
#endif
  }
  return NULL;
}


const std::string getFunctionName(const std::string &name)
{
    size_t end = name.rfind('(');
    if(end == std::string::npos)
    {
        end = name.size();
    }
    size_t begin = 1 + name.rfind(' ', end);
    return name.substr(begin, end - begin);
}
