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

#include <fstream>

#include <kinect2_registration/kinect2_console.h>

#include <CL/opencl.hpp>

#ifndef REG_OPENCL_FILE
#define REG_OPENCL_FILE ""
#endif

#include "depth_registration_opencl.h"

//#define ENABLE_PROFILING_CL

#define CL_FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define PRINT_CL_ERROR(logger, expr, err) OUT_ERROR(logger, FG_BLUE "[" << CL_FILENAME << "]" FG_CYAN "(" << __LINE__ << ") " FG_YELLOW << expr << FG_RED " failed: " << err)

#define CHECK_CL_PARAM(logger, expr) do { cl_int err = CL_SUCCESS; (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(logger, #expr, err); return false; } } while(0)
#define CHECK_CL_RETURN(logger, expr) do { cl_int err = (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(logger, #expr, err); return false; } } while(0)
#define CHECK_CL_ON_FAIL(logger, expr, on_fail) do { cl_int err = (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(logger, #expr, err); on_fail; return false; } } while(0)

struct DepthRegistrationOpenCL::OCLData
{
  cl::Context context;
  cl::Device device;

  cl::Program program;
  cl::CommandQueue queue;

  cl::Kernel kernelSetZero;
  cl::Kernel kernelProject;
  cl::Kernel kernelCheckDepth;
  cl::Kernel kernelRemap;

  size_t sizeDepth;
  size_t sizeRegistered;
  size_t sizeIndex;
  size_t sizeImgZ;
  size_t sizeDists;
  size_t sizeSelDist;
  size_t sizeMap;

  cl::Buffer bufferDepth;
  cl::Buffer bufferScaled;
  cl::Buffer bufferRegistered;
  cl::Buffer bufferIndex;
  cl::Buffer bufferImgZ;
  cl::Buffer bufferDists;
  cl::Buffer bufferSelDist;
  cl::Buffer bufferMapX;
  cl::Buffer bufferMapY;

  cl::Buffer bufferOutput;
  unsigned char *dataOutput;

#ifdef ENABLE_PROFILING_CL
  std::vector<double> timings;
  int count;
#endif
};

DepthRegistrationOpenCL::DepthRegistrationOpenCL()
  : DepthRegistration()
{
  data = new OCLData;
}

DepthRegistrationOpenCL::~DepthRegistrationOpenCL()
{
  delete data;
}

void getDevices(const std::vector<cl::Platform> &platforms, std::vector<cl::Device> &devices)
{
  devices.clear();
  for(size_t i = 0; i < platforms.size(); ++i)
  {
    const cl::Platform &platform = platforms[i];

    std::vector<cl::Device> devs;
    if(platform.getDevices(CL_DEVICE_TYPE_ALL, &devs) != CL_SUCCESS)
    {
      continue;
    }

    devices.insert(devices.end(), devs.begin(), devs.end());
  }
}

std::string deviceString(cl::Device &dev)
{
  std::string devName, devVendor, devType;
  cl_device_type devTypeID;
  dev.getInfo(CL_DEVICE_NAME, &devName);
  dev.getInfo(CL_DEVICE_VENDOR, &devVendor);
  dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

  switch(devTypeID)
  {
  case CL_DEVICE_TYPE_CPU:
    devType = "CPU";
    break;
  case CL_DEVICE_TYPE_GPU:
    devType = "GPU";
    break;
  case CL_DEVICE_TYPE_ACCELERATOR:
    devType = "ACCELERATOR";
    break;
  default:
    devType = "CUSTOM/UNKNOWN";
  }

  return devName + " (" + devType + ")[" + devVendor + ']';
}

bool selectDevice(std::vector<cl::Device> &devices, cl::Device &device, const int deviceId = -1)
{
  if(deviceId != -1 && devices.size() > (size_t)deviceId)
  {
    device = devices[deviceId];
    return true;
  }

  bool selected = false;
  cl_device_type selectedType = 0;

  for(size_t i = 0; i < devices.size(); ++i)
  {
    cl::Device &dev = devices[i];
    cl_device_type devTypeID;
    dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

    if(!selected || (selectedType != CL_DEVICE_TYPE_GPU && devTypeID == CL_DEVICE_TYPE_GPU))
    {
      selectedType = devTypeID;
      selected = true;
      device = dev;
    }
  }
  return selected;
}

bool DepthRegistrationOpenCL::init(rclcpp::Logger& logger, const int deviceId)
{
  std::string sourceCode;
  if(!readProgram(sourceCode))
  {
    return false;
  }

  std::vector<cl::Platform> platforms;
  CHECK_CL_RETURN(logger, cl::Platform::get(&platforms));

  if(platforms.empty())
  {
    OUT_ERROR(logger, "no opencl platforms found.");
    return false;
  }

  std::vector<cl::Device> devices;
  getDevices(platforms, devices);

  OUT_INFO(logger, "devices:");
  for(size_t i = 0; i < devices.size(); ++i)
  {
    OUT_INFO(logger, "  " << i << ": " FG_CYAN << deviceString(devices[i]) << NO_COLOR);
  }

  if(!selectDevice(devices, data->device, deviceId))
  {
    OUT_ERROR(logger, "could not find any suitable device");
    return false;
  }
  OUT_INFO(logger, "selected device: " FG_YELLOW << deviceString(data->device) << NO_COLOR);

  CHECK_CL_PARAM(logger, data->context = cl::Context(data->device, NULL, NULL, NULL, &err));

  std::string options;
  generateOptions(options);

  cl::Program::Sources source(1, std::make_pair(sourceCode.c_str(), sourceCode.length()));
  CHECK_CL_PARAM(logger, data->program = cl::Program(data->context, source, &err));

  CHECK_CL_ON_FAIL(logger, data->program.build(options.c_str()),
                   OUT_ERROR(logger, "failed to build program: " << err);
                   OUT_ERROR(logger, "Build Status: " << data->program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(data->device));
                   OUT_ERROR(logger, "Build Options:\t" << data->program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(data->device));
                   OUT_ERROR(logger, "Build Log:\t " << data->program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(data->device)));

#ifdef ENABLE_PROFILING_CL
  data->count = 0;
  CHECK_CL_PARAM(logger, data->queue = cl::CommandQueue(data->context, data->device, CL_QUEUE_PROFILING_ENABLE, &err));
#else
  CHECK_CL_PARAM(logger, data->queue = cl::CommandQueue(data->context, data->device, 0, &err));
#endif

  data->sizeDepth = sizeDepth.height * sizeDepth.width * sizeof(uint16_t);
  data->sizeRegistered = sizeRegistered.height * sizeRegistered.width * sizeof(uint16_t);
  data->sizeIndex = sizeRegistered.height * sizeRegistered.width * sizeof(cl_int4);
  data->sizeImgZ = sizeRegistered.height * sizeRegistered.width * sizeof(uint16_t);
  data->sizeDists = sizeRegistered.height * sizeRegistered.width * sizeof(cl_float4);
  data->sizeSelDist = sizeRegistered.height * sizeRegistered.width * sizeof(float);
  data->sizeMap = sizeRegistered.height * sizeRegistered.width * sizeof(float);

  CHECK_CL_PARAM(logger, data->bufferDepth = cl::Buffer(data->context, CL_MEM_READ_ONLY, data->sizeDepth, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferScaled = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeRegistered, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferRegistered = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeRegistered, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferIndex = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeIndex, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferImgZ = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeImgZ, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferDists = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeDists, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferSelDist = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizeSelDist, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferMapX = cl::Buffer(data->context, CL_MEM_READ_ONLY, data->sizeMap, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferMapY = cl::Buffer(data->context, CL_MEM_READ_ONLY, data->sizeMap, NULL, &err));
  CHECK_CL_PARAM(logger, data->bufferOutput = cl::Buffer(data->context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR, data->sizeRegistered, NULL, &err));

  CHECK_CL_PARAM(logger, data->kernelSetZero = cl::Kernel(data->program, "setZero", &err));
  CHECK_CL_RETURN(logger, data->kernelSetZero.setArg(0, data->bufferRegistered));
  CHECK_CL_RETURN(logger, data->kernelSetZero.setArg(1, data->bufferSelDist));

  CHECK_CL_PARAM(logger, data->kernelProject = cl::Kernel(data->program, "project", &err));
  CHECK_CL_RETURN(logger, data->kernelProject.setArg(0, data->bufferScaled));
  CHECK_CL_RETURN(logger, data->kernelProject.setArg(1, data->bufferIndex));
  CHECK_CL_RETURN(logger, data->kernelProject.setArg(2, data->bufferImgZ));
  CHECK_CL_RETURN(logger, data->kernelProject.setArg(3, data->bufferDists));
  CHECK_CL_RETURN(logger, data->kernelProject.setArg(4, data->bufferSelDist));
  CHECK_CL_RETURN(logger, data->kernelProject.setArg(5, data->bufferRegistered));

  CHECK_CL_PARAM(logger, data->kernelCheckDepth = cl::Kernel(data->program, "checkDepth", &err));
  CHECK_CL_RETURN(logger, data->kernelCheckDepth.setArg(0, data->bufferIndex));
  CHECK_CL_RETURN(logger, data->kernelCheckDepth.setArg(1, data->bufferImgZ));
  CHECK_CL_RETURN(logger, data->kernelCheckDepth.setArg(2, data->bufferDists));
  CHECK_CL_RETURN(logger, data->kernelCheckDepth.setArg(3, data->bufferSelDist));
  CHECK_CL_RETURN(logger, data->kernelCheckDepth.setArg(4, data->bufferRegistered));

  CHECK_CL_PARAM(logger, data->kernelRemap = cl::Kernel(data->program, "remapDepth", &err));
  CHECK_CL_RETURN(logger, data->kernelRemap.setArg(0, data->bufferDepth));
  CHECK_CL_RETURN(logger, data->kernelRemap.setArg(1, data->bufferScaled));
  CHECK_CL_RETURN(logger, data->kernelRemap.setArg(2, data->bufferMapX));
  CHECK_CL_RETURN(logger, data->kernelRemap.setArg(3, data->bufferMapY));

  CHECK_CL_RETURN(logger, data->queue.enqueueWriteBuffer(data->bufferMapX, CL_TRUE, 0, data->sizeMap, mapX.data));
  CHECK_CL_RETURN(logger, data->queue.enqueueWriteBuffer(data->bufferMapY, CL_TRUE, 0, data->sizeMap, mapY.data));

  CHECK_CL_PARAM(logger, data->dataOutput = (unsigned char *)data->queue.enqueueMapBuffer(data->bufferOutput, CL_TRUE, CL_MAP_READ, 0, data->sizeRegistered, NULL, NULL, &err));
  return true;
}

bool DepthRegistrationOpenCL::registerDepth(rclcpp::Logger& logger, const cv::Mat &depth, cv::Mat &registered)
{
  cl::Event eventRead;
  std::vector<cl::Event> eventZero(2), eventRemap(1), eventProject(1), eventCheckDepth1(1), eventCheckDepth2(1);
  cl::NDRange range(sizeRegistered.height * sizeRegistered.width);

  CHECK_CL_RETURN(logger, data->queue.enqueueWriteBuffer(data->bufferDepth, CL_FALSE, 0, data->sizeDepth, depth.data, NULL, &eventZero[0]));
  CHECK_CL_RETURN(logger, data->queue.enqueueNDRangeKernel(data->kernelSetZero, cl::NullRange, range, cl::NullRange, NULL, &eventZero[1]));

  CHECK_CL_RETURN(logger, data->queue.enqueueNDRangeKernel(data->kernelRemap, cl::NullRange, range, cl::NullRange, &eventZero, &eventRemap[0]));

  CHECK_CL_RETURN(logger, data->queue.enqueueNDRangeKernel(data->kernelProject, cl::NullRange, range, cl::NullRange, &eventRemap, &eventProject[0]));

  CHECK_CL_RETURN(logger, data->queue.enqueueNDRangeKernel(data->kernelCheckDepth, cl::NullRange, range, cl::NullRange, &eventProject, &eventCheckDepth1[0]));

  CHECK_CL_RETURN(logger, data->queue.enqueueNDRangeKernel(data->kernelCheckDepth, cl::NullRange, range, cl::NullRange, &eventCheckDepth1, &eventCheckDepth2[0]));

  CHECK_CL_RETURN(logger, data->queue.enqueueReadBuffer(data->bufferRegistered, CL_FALSE, 0, data->sizeRegistered, data->dataOutput, &eventCheckDepth2, &eventRead));

  CHECK_CL_RETURN(logger, eventRead.wait());

  registered = cv::Mat(sizeRegistered, CV_16U, data->dataOutput);

#ifdef ENABLE_PROFILING_CL
  if(data->count == 0)
  {
    data->timings.clear();
    data->timings.resize(7, 0.0);
  }

  data->timings[0] += eventZero[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventZero[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[1] += eventZero[1].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventZero[1].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[2] += eventRemap[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventRemap[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[3] += eventProject[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventProject[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[4] += eventCheckDepth1[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventCheckDepth1[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[5] += eventCheckDepth2[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventCheckDepth2[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[6] += eventRead.getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventRead.getProfilingInfo<CL_PROFILING_COMMAND_START>();

  if(++data->count == 100)
  {
    double sum = data->timings[0] + data->timings[1] + data->timings[2] + data->timings[3] + data->timings[4] + data->timings[5] + data->timings[6];
    OUT_INFO(logger, "writing depth: " << data->timings[0] / 100000000.0 << " ms.");
    OUT_INFO(logger, "setting zero: " << data->timings[1] / 100000000.0 << " ms.");
    OUT_INFO(logger, "remap: " << data->timings[2] / 100000000.0 << " ms.");
    OUT_INFO(logger, "project: " << data->timings[3] / 100000000.0 << " ms.");
    OUT_INFO(logger, "check depth 1: " << data->timings[4] / 100000000.0 << " ms.");
    OUT_INFO(logger, "check depth 2: " << data->timings[5] / 100000000.0 << " ms.");
    OUT_INFO(logger, "read registered: " << data->timings[6] / 100000000.0 << " ms.");
    OUT_INFO(logger, "overall: " << sum / 100000000.0 << " ms.");
    data->count = 0;
  }
#endif
  return true;
}

void DepthRegistrationOpenCL::generateOptions(std::string &options) const
{
  std::ostringstream oss;
  oss.precision(16);
  oss << std::scientific;

  // Rotation
  oss << " -D r00=" << rotation.at<double>(0, 0) << "f";
  oss << " -D r01=" << rotation.at<double>(0, 1) << "f";
  oss << " -D r02=" << rotation.at<double>(0, 2) << "f";
  oss << " -D r10=" << rotation.at<double>(1, 0) << "f";
  oss << " -D r11=" << rotation.at<double>(1, 1) << "f";
  oss << " -D r12=" << rotation.at<double>(1, 2) << "f";
  oss << " -D r20=" << rotation.at<double>(2, 0) << "f";
  oss << " -D r21=" << rotation.at<double>(2, 1) << "f";
  oss << " -D r22=" << rotation.at<double>(2, 2) << "f";

  // Translation
  oss << " -D tx=" << translation.at<double>(0, 0) << "f";
  oss << " -D ty=" << translation.at<double>(1, 0) << "f";
  oss << " -D tz=" << translation.at<double>(2, 0) << "f";

  // Camera parameter upscaled depth
  oss << " -D fxR=" << cameraMatrixRegistered.at<double>(0, 0) << "f";
  oss << " -D fyR=" << cameraMatrixRegistered.at<double>(1, 1) << "f";
  oss << " -D cxR=" << cameraMatrixRegistered.at<double>(0, 2) << "f";
  oss << " -D cyR=" << cameraMatrixRegistered.at<double>(1, 2) << "f";
  oss << " -D fxRInv=" << (1.0 / cameraMatrixRegistered.at<double>(0, 0)) << "f";
  oss << " -D fyRInv=" << (1.0 / cameraMatrixRegistered.at<double>(1, 1)) << "f";

  // Clipping distances
  oss << " -D zNear=" << (uint16_t)(zNear * 1000);
  oss << " -D zFar=" << (uint16_t)(zFar * 1000);

  // Size registered image
  oss << " -D heightR=" << sizeRegistered.height;
  oss << " -D widthR=" << sizeRegistered.width;

  // Size depth image
  oss << " -D heightD=" << sizeDepth.height;
  oss << " -D widthD=" << sizeDepth.width;

  options = oss.str();
}

bool DepthRegistrationOpenCL::readProgram(std::string &source) const
{
  std::ifstream file(REG_OPENCL_FILE);

  if(!file.is_open())
  {
    return false;
  }

  source = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  return true;
}
