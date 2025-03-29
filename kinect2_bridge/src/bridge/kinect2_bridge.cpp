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

#include "kinect2_bridge.h"

#include "kinect2_bridge/kinect2_definitions.h"
#include "utils/time_utils.h"

#include <compressed_depth_image_transport/compression_common.hpp>
#include <kinect2_registration/kinect2_console.h>
#include <kinect2_registration/kinect2_registration.h>
#include <libfreenect2/config.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
//#include <tf/transform_broadcaster.h> // TODO: tf2

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#if defined(__linux__)
#include <sys/prctl.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

using namespace std::chrono_literals;

Kinect2Bridge::Kinect2Bridge(rclcpp::Node& node)
  : node(node), sizeColor(1920, 1080), sizeIr(512, 424), sizeLowRes(sizeColor.width / 2, sizeColor.height / 2),
    color(sizeColor.width, sizeColor.height, 4)
{
  // Initialize member variables
  status.resize(COUNT, UNSUBCRIBED);

  // Declare ROS parameters
  this->node.declare_parameter<std::string>("base_name", K2_DEFAULT_NS);
  this->node.declare_parameter<std::string>("sensor", "");
  this->node.declare_parameter<double>("fps_limit", -1.0);
  this->node.declare_parameter<std::string>("calib_path", K2_CALIB_PATH);
  this->node.declare_parameter<bool>("use_png", false);
  this->node.declare_parameter<int>("jpeg_quality", 90);
  this->node.declare_parameter<int>("png_level", 1);
  this->node.declare_parameter<std::string>("depth_method", "cpu");
  this->node.declare_parameter<int>("depth_device", -1);
  this->node.declare_parameter<std::string>("reg_method", "default");
  this->node.declare_parameter<int>("reg_device", -1);
  this->node.declare_parameter<double>("max_depth", 12.0);
  this->node.declare_parameter<double>("min_depth", 0.1);
  this->node.declare_parameter<int>("queue_size", 2);
  this->node.declare_parameter<bool>("bilateral_filter", true);
  this->node.declare_parameter<bool>("edge_aware_filter", true);
  this->node.declare_parameter<bool>("publish_tf", false);
  this->node.declare_parameter<std::string>("base_name_tf", K2_DEFAULT_NS);
  this->node.declare_parameter<int>("worker_threads", 4);
}

bool Kinect2Bridge::start()
{
  if(running)
  {
    OUT_ERROR(node.get_logger(), "kinect2_bridge is already running!");
    return false;
  }
  if(!initialize())
  {
    OUT_ERROR(node.get_logger(), "Initialization failed!");
    return false;
  }
  running = true;

  if(publishTF)
  {
    tfPublisher = std::thread(&Kinect2Bridge::publishStaticTF, this);
  }

  for(size_t i = 0; i < threads.size(); ++i)
  {
    threads[i] = std::thread(&Kinect2Bridge::threadDispatcher, this, i);
  }

  setupMainThread();

  return true;
}

void Kinect2Bridge::stop()
{
  if(!running)
  {
    OUT_ERROR(node.get_logger(), "kinect2_bridge is not running!");
    return;
  }
  running = false;

  checkTimer->cancel();
  mainTimer->cancel();

  checkTimer->reset();
  mainTimer->reset();

  for(size_t i = 0; i < threads.size(); ++i)
  {
    threads[i].join();
  }

  if(publishTF)
  {
    tfPublisher.join();
  }

  if(deviceActive && !device->stop())
  {
    OUT_ERROR(node.get_logger(), "could not stop device!");
  }

  if(!device->close())
  {
    OUT_ERROR(node.get_logger(), "could not close device!");
  }

  delete listenerIrDepth;
  delete listenerColor;
  delete registration;

  delete depthRegLowRes;
  delete depthRegHighRes;

  imagePubs.clear();
  compressedPubs.clear();
  infoHDPub.reset();
  infoQHDPub.reset();
  infoIRPub.reset();
}

bool Kinect2Bridge::initialize()
{
  double fps_limit, maxDepth, minDepth;
  bool use_png, bilateral_filter, edge_aware_filter;
  int32_t jpeg_quality, png_level, queueSize, reg_dev, depth_dev, worker_threads;
  std::string depth_method, reg_method, calib_path, sensor, base_name;

  std::string depthDefault = "cpu";
  std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
  depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
  depthDefault = "opencl";
#endif
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
  depthDefault = "cuda";
#endif
#ifdef DEPTH_REG_OPENCL
  regDefault = "opencl";
#endif

  node.get_parameter_or("base_name", base_name, std::string(K2_DEFAULT_NS));
  node.get_parameter_or("sensor", sensor, std::string(""));
  node.get_parameter_or("fps_limit", fps_limit, -1.0);
  node.get_parameter_or("calib_path", calib_path, std::string(K2_CALIB_PATH));
  node.get_parameter_or("use_png", use_png, false);
  node.get_parameter_or("jpeg_quality", jpeg_quality, 90);
  node.get_parameter_or("png_level", png_level, 1);
  node.get_parameter_or("depth_method", depth_method, depthDefault);
  node.get_parameter_or("depth_device", depth_dev, -1);
  node.get_parameter_or("reg_method", reg_method, regDefault);
  node.get_parameter_or("reg_device", reg_dev, -1);
  node.get_parameter_or("max_depth", maxDepth, 12.0);
  node.get_parameter_or("min_depth", minDepth, 0.1);
  node.get_parameter_or("queue_size", queueSize, 2);
  node.get_parameter_or("bilateral_filter", bilateral_filter, true);
  node.get_parameter_or("edge_aware_filter", edge_aware_filter, true);
  node.get_parameter_or("publish_tf", publishTF, false);
  node.get_parameter_or("base_name_tf", baseNameTF, base_name);
  node.get_parameter_or("worker_threads", worker_threads, 4);

  worker_threads = std::max(1, worker_threads);
  threads.resize(worker_threads);

  OUT_INFO(node.get_logger(), "parameter:" << std::endl
           << "        base_name: " FG_CYAN << base_name << NO_COLOR << std::endl
           << "           sensor: " FG_CYAN << (sensor.empty() ? "default" : sensor) << NO_COLOR << std::endl
           << "        fps_limit: " FG_CYAN << fps_limit << NO_COLOR << std::endl
           << "       calib_path: " FG_CYAN << calib_path << NO_COLOR << std::endl
           << "          use_png: " FG_CYAN << (use_png ? "true" : "false") << NO_COLOR << std::endl
           << "     jpeg_quality: " FG_CYAN << jpeg_quality << NO_COLOR << std::endl
           << "        png_level: " FG_CYAN << png_level << NO_COLOR << std::endl
           << "     depth_method: " FG_CYAN << depth_method << NO_COLOR << std::endl
           << "     depth_device: " FG_CYAN << depth_dev << NO_COLOR << std::endl
           << "       reg_method: " FG_CYAN << reg_method << NO_COLOR << std::endl
           << "       reg_device: " FG_CYAN << reg_dev << NO_COLOR << std::endl
           << "        max_depth: " FG_CYAN << maxDepth << NO_COLOR << std::endl
           << "        min_depth: " FG_CYAN << minDepth << NO_COLOR << std::endl
           << "       queue_size: " FG_CYAN << queueSize << NO_COLOR << std::endl
           << " bilateral_filter: " FG_CYAN << (bilateral_filter ? "true" : "false") << NO_COLOR << std::endl
           << "edge_aware_filter: " FG_CYAN << (edge_aware_filter ? "true" : "false") << NO_COLOR << std::endl
           << "       publish_tf: " FG_CYAN << (publishTF ? "true" : "false") << NO_COLOR << std::endl
           << "     base_name_tf: " FG_CYAN << baseNameTF << NO_COLOR << std::endl
           << "   worker_threads: " FG_CYAN << worker_threads << NO_COLOR);

  deltaT = std::chrono::duration_cast<std::chrono::nanoseconds>(fps_limit > 0 ? 1s / fps_limit : 0s);

  if(calib_path.empty() || calib_path.back() != '/')
  {
    calib_path += '/';
  }

  initCompression(jpeg_quality, png_level, use_png);

  if(!initPipeline(depth_method, depth_dev))
  {
    return false;
  }

  if(!initDevice(sensor))
  {
    return false;
  }

  initConfig(bilateral_filter, edge_aware_filter, minDepth, maxDepth);

  initCalibration(calib_path, sensor);

  if(!initRegistration(reg_method, reg_dev, maxDepth))
  {
    if(!device->close())
    {
      OUT_ERROR(node.get_logger(), "could not close device!");
    }
    delete listenerIrDepth;
    delete listenerColor;
    return false;
  }

  createCameraInfo();
  initTopics(queueSize, base_name);

  return true;
}

bool Kinect2Bridge::initRegistration(const std::string &method, const int32_t device, const double maxDepth)
{
  DepthRegistration::Method reg;

  if(method == "default")
  {
    reg = DepthRegistration::DEFAULT;
  }
  else if(method == "cpu")
  {
#ifdef DEPTH_REG_CPU
    reg = DepthRegistration::CPU;
#else
    OUT_ERROR(node.get_logger(), "CPU registration is not available!");
    return false;
#endif
  }
  else if(method == "opencl")
  {
#ifdef DEPTH_REG_OPENCL
    reg = DepthRegistration::OPENCL;
#else
    OUT_ERROR(node.get_logger(), "OpenCL registration is not available!");
    return false;
#endif
  }
  else
  {
    OUT_ERROR(node.get_logger(), "Unknown registration method: " << method);
    return false;
  }

  depthRegLowRes = DepthRegistration::New(node.get_logger(), reg);
  depthRegHighRes = DepthRegistration::New(node.get_logger(), reg);

  if(!depthRegLowRes->init(cameraMatrixLowRes, sizeLowRes, cameraMatrixDepth, sizeIr, distortionDepth, rotation, translation, 0.5f, maxDepth, device) ||
     !depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixDepth, sizeIr, distortionDepth, rotation, translation, 0.5f, maxDepth, device))
  {
    delete depthRegLowRes;
    delete depthRegHighRes;
    return false;
  }

  registration = new libfreenect2::Registration(irParams, colorParams);

  return true;
}

bool Kinect2Bridge::initPipeline(const std::string &method, const int32_t device)
{
  if(method == "default")
  {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    packetPipeline = new libfreenect2::CudaPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
    packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
    packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
    packetPipeline = new libfreenect2::CpuPacketPipeline();
#endif
  }
  else if(method == "cpu")
  {
    packetPipeline = new libfreenect2::CpuPacketPipeline();
  }
  else if(method == "cuda")
  {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    packetPipeline = new libfreenect2::CudaPacketPipeline(device);
#else
    OUT_ERROR(node.get_logger(), "Cuda depth processing is not available!");
    return false;
#endif
  }
  else if(method == "opencl")
  {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#else
    OUT_ERROR(node.get_logger(), "OpenCL depth processing is not available!");
    return false;
#endif
  }
  else if(method == "opengl")
  {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
    OUT_ERROR(node.get_logger(), "OpenGL depth processing is not available!");
    return false;
#endif
  }
  else if(method == "clkde")
  {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    packetPipeline = new libfreenect2::OpenCLKdePacketPipeline(device);
#else
    OUT_ERROR(node.get_logger(), "OpenCL depth processing is not available!");
    return false;
#endif
  }
  else if(method == "cudakde")
  {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    packetPipeline = new libfreenect2::CudaKdePacketPipeline(device);
#else
    OUT_ERROR(node.get_logger(), "Cuda depth processing is not available!");
    return false;
#endif
  }
  else
  {
    OUT_ERROR(node.get_logger(), "Unknown depth processing method: " << method);
    return false;
  }

  return true;
}

void Kinect2Bridge::initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
{
  libfreenect2::Freenect2Device::Config config;
  config.EnableBilateralFilter = bilateral_filter;
  config.EnableEdgeAwareFilter = edge_aware_filter;
  config.MinDepth = minDepth;
  config.MaxDepth = maxDepth;
  device->setConfiguration(config);
}

void Kinect2Bridge::initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png)
{
  compressionParams.resize(7, 0);
  compressionParams[0] = cv::IMWRITE_JPEG_QUALITY;
  compressionParams[1] = jpegQuality;
  compressionParams[2] = cv::IMWRITE_PNG_COMPRESSION;
  compressionParams[3] = pngLevel;
  compressionParams[4] = cv::IMWRITE_PNG_STRATEGY;
  compressionParams[5] = cv::IMWRITE_PNG_STRATEGY_RLE;
  compressionParams[6] = 0;

  if(use_png)
  {
    compression16BitExt = ".png";
    compression16BitString = std::string(sensor_msgs::image_encodings::TYPE_16UC1) + "; png compressed";
  }
  else
  {
    compression16BitExt = ".tif";
    compression16BitString = std::string(sensor_msgs::image_encodings::TYPE_16UC1) + "; tiff compressed";
  }
}

void Kinect2Bridge::initTopics(const int32_t queueSize, const std::string &base_name)
{
  std::vector<std::string> topics(COUNT);
  topics[IR_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR;
  topics[IR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;

  topics[DEPTH_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH;
  topics[DEPTH_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  topics[DEPTH_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  topics[DEPTH_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

  topics[COLOR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;

  topics[COLOR_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR;
  topics[COLOR_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  topics[COLOR_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR;
  topics[COLOR_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;

  topics[MONO_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO;
  topics[MONO_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;
  topics[MONO_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO;
  topics[MONO_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;

  imagePubs.reserve(COUNT);
  compressedPubs.reserve(COUNT);
  auto cbStatus = std::bind(&Kinect2Bridge::callbackStatus, this);
  auto cbMain = std::bind(&Kinect2Bridge::main, this);

  using ImgMsg = sensor_msgs::msg::Image;
  using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

  for(size_t i = 0; i < COUNT; ++i)
  {
    imagePubs.emplace_back(node.create_publisher<ImgMsg>(base_name + topics[i], queueSize));
    compressedPubs.emplace_back(node.create_publisher<CompressedImageMsg>(base_name + topics[i] + K2_TOPIC_COMPRESSED, queueSize));
  }
  infoHDPub = node.create_publisher<CameraInfoMsg>(base_name + K2_TOPIC_HD + K2_TOPIC_INFO, queueSize);
  infoQHDPub = node.create_publisher<CameraInfoMsg>(base_name + K2_TOPIC_QHD + K2_TOPIC_INFO, queueSize);
  infoIRPub = node.create_publisher<CameraInfoMsg>(base_name + K2_TOPIC_SD + K2_TOPIC_INFO, queueSize);

  checkTimer = node.create_wall_timer(100ms, cbStatus);
  mainTimer = node.create_wall_timer(10ms, cbMain);
}

bool Kinect2Bridge::initDevice(std::string &sensor)
{
  bool deviceFound = false;
  const int numOfDevs = freenect2.enumerateDevices();

  if(numOfDevs <= 0)
  {
    OUT_ERROR(node.get_logger(), "no Kinect2 devices found!");
    delete packetPipeline;
    return false;
  }

  if(sensor.empty())
  {
    sensor = freenect2.getDefaultDeviceSerialNumber();
  }

  OUT_INFO(node.get_logger(), "Kinect2 devices found: ");
  for(int i = 0; i < numOfDevs; ++i)
  {
    const std::string &s = freenect2.getDeviceSerialNumber(i);
    deviceFound = deviceFound || s == sensor;
    OUT_INFO(node.get_logger(), "  " << i << ": " FG_CYAN << s << (s == sensor ? FG_YELLOW " (selected)" : "") << NO_COLOR);
  }

  if(!deviceFound)
  {
    OUT_ERROR(node.get_logger(), "Device with serial '" << sensor << "' not found!");
    delete packetPipeline;
    return false;
  }

  device = freenect2.openDevice(sensor, packetPipeline);

  if(device == 0)
  {
    OUT_INFO(node.get_logger(), "no device connected or failure opening the default one!");
    return false;
  }

  listenerColor = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
  listenerIrDepth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

  device->setColorFrameListener(listenerColor);
  device->setIrAndDepthFrameListener(listenerIrDepth);

  OUT_INFO(node.get_logger(), "starting kinect2");
  if(!device->start())
  {
    OUT_ERROR(node.get_logger(), "could not start device!");
    delete listenerIrDepth;
    delete listenerColor;
    return false;
  }

  OUT_INFO(node.get_logger(), "device serial: " FG_CYAN << sensor << NO_COLOR);
  OUT_INFO(node.get_logger(), "device firmware: " FG_CYAN << device->getFirmwareVersion() << NO_COLOR);

  colorParams = device->getColorCameraParams();
  irParams = device->getIrCameraParams();

  if(!device->stop())
  {
    OUT_ERROR(node.get_logger(), "could not stop device!");
    delete listenerIrDepth;
    delete listenerColor;
    return false;
  }

  OUT_DEBUG(node.get_logger(), "default ir camera parameters: ");
  OUT_DEBUG(node.get_logger(), "fx: " FG_CYAN << irParams.fx << NO_COLOR ", fy: " FG_CYAN << irParams.fy << NO_COLOR ", cx: " FG_CYAN << irParams.cx << NO_COLOR ", cy: " FG_CYAN << irParams.cy << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "k1: " FG_CYAN << irParams.k1 << NO_COLOR ", k2: " FG_CYAN << irParams.k2 << NO_COLOR ", p1: " FG_CYAN << irParams.p1 << NO_COLOR ", p2: " FG_CYAN << irParams.p2 << NO_COLOR ", k3: " FG_CYAN << irParams.k3 << NO_COLOR);

  OUT_DEBUG(node.get_logger(), "default color camera parameters: ");
  OUT_DEBUG(node.get_logger(), "fx: " FG_CYAN << colorParams.fx << NO_COLOR ", fy: " FG_CYAN << colorParams.fy << NO_COLOR ", cx: " FG_CYAN << colorParams.cx << NO_COLOR ", cy: " FG_CYAN << colorParams.cy << NO_COLOR);

  cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
  distortionColor = cv::Mat::zeros(1, 5, CV_64F);

  cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
  cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
  cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
  cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
  cameraMatrixColor.at<double>(2, 2) = 1;

  cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
  distortionIr = cv::Mat::zeros(1, 5, CV_64F);

  cameraMatrixIr.at<double>(0, 0) = irParams.fx;
  cameraMatrixIr.at<double>(1, 1) = irParams.fy;
  cameraMatrixIr.at<double>(0, 2) = irParams.cx;
  cameraMatrixIr.at<double>(1, 2) = irParams.cy;
  cameraMatrixIr.at<double>(2, 2) = 1;

  distortionIr.at<double>(0, 0) = irParams.k1;
  distortionIr.at<double>(0, 1) = irParams.k2;
  distortionIr.at<double>(0, 2) = irParams.p1;
  distortionIr.at<double>(0, 3) = irParams.p2;
  distortionIr.at<double>(0, 4) = irParams.k3;

  cameraMatrixDepth = cameraMatrixIr.clone();
  distortionDepth = distortionIr.clone();

  rotation = cv::Mat::eye(3, 3, CV_64F);
  translation = cv::Mat::zeros(3, 1, CV_64F);
  return true;
}

void Kinect2Bridge::initCalibration(const std::string &calib_path, const std::string &sensor)
{
  std::string calibPath = calib_path + sensor + '/';

  struct stat fileStat;
  bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
  if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
  {
    OUT_WARN(node.get_logger(), "using sensor defaults for color intrinsic parameters.");
  }

  if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixDepth, distortionDepth))
  {
    OUT_WARN(node.get_logger(), "using sensor defaults for ir intrinsic parameters.");
  }

  if(calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
  {
    OUT_WARN(node.get_logger(), "using defaults for rotation and translation.");
  }

  if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
  {
    OUT_WARN(node.get_logger(), "using defaults for depth shift.");
    depthShift = std::chrono::nanoseconds{};
  }

  cameraMatrixLowRes = cameraMatrixColor.clone();
  cameraMatrixLowRes.at<double>(0, 0) /= 2;
  cameraMatrixLowRes.at<double>(1, 1) /= 2;
  cameraMatrixLowRes.at<double>(0, 2) /= 2;
  cameraMatrixLowRes.at<double>(1, 2) /= 2;

  const int mapType = CV_16SC2;
  cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
  cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
  cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);

  OUT_DEBUG(node.get_logger(), "camera parameters used:");
  OUT_DEBUG(node.get_logger(), "camera matrix color:" FG_CYAN << std::endl << cameraMatrixColor << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "distortion coefficients color:" FG_CYAN << std::endl << distortionColor << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "camera matrix ir:" FG_CYAN << std::endl << cameraMatrixIr << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "distortion coefficients ir:" FG_CYAN << std::endl << distortionIr << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "camera matrix depth:" FG_CYAN << std::endl << cameraMatrixDepth << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "distortion coefficients depth:" FG_CYAN << std::endl << distortionDepth << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "rotation:" FG_CYAN << std::endl << rotation << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "translation:" FG_CYAN << std::endl << translation << NO_COLOR);
  OUT_DEBUG(node.get_logger(), "depth shift:" FG_CYAN << std::endl << depthShift.count() << NO_COLOR);
}

bool Kinect2Bridge::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
    fs[K2_CALIB_DISTORTION] >> distortion;
    fs.release();
  }
  else
  {
    OUT_ERROR(node.get_logger(), "can't open calibration file: " << filename);
    return false;
  }
  return true;
}

bool Kinect2Bridge::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_ROTATION] >> rotation;
    fs[K2_CALIB_TRANSLATION] >> translation;
    fs.release();
  }
  else
  {
    OUT_ERROR(node.get_logger(), "can't open calibration pose file: " << filename);
    return false;
  }
  return true;
}

bool Kinect2Bridge::loadCalibrationDepthFile(const std::string &filename, std::chrono::nanoseconds &depthShift) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    double _depthShift;
    fs[K2_CALIB_DEPTH_SHIFT] >> _depthShift;
    depthShift = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(_depthShift));
    fs.release();
  }
  else
  {
    OUT_ERROR(node.get_logger(), "can't open calibration depth file: " << filename);
    return false;
  }
  return true;
}

void Kinect2Bridge::createCameraInfo()
{
  cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F);
  cv::Mat projIr = cv::Mat::zeros(3, 4, CV_64F);
  cv::Mat projLowRes = cv::Mat::zeros(3, 4, CV_64F);

  cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
  cameraMatrixIr.copyTo(projIr(cv::Rect(0, 0, 3, 3)));
  cameraMatrixLowRes.copyTo(projLowRes(cv::Rect(0, 0, 3, 3)));

  createCameraInfo(sizeColor, cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, infoHD);
  createCameraInfo(sizeIr, cameraMatrixIr, distortionIr, cv::Mat::eye(3, 3, CV_64F), projIr, infoIR);
  createCameraInfo(sizeLowRes, cameraMatrixLowRes, distortionColor, cv::Mat::eye(3, 3, CV_64F), projLowRes, infoQHD);
}

void Kinect2Bridge::createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::msg::CameraInfo &cameraInfo) const
{
  cameraInfo.height = size.height;
  cameraInfo.width = size.width;

  const double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    cameraInfo.k[i] = *itC;
  }

  const double *itR = rotation.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itR)
  {
    cameraInfo.r[i] = *itR;
  }

  const double *itP = projection.ptr<double>(0, 0);
  for(size_t i = 0; i < 12; ++i, ++itP)
  {
    cameraInfo.p[i] = *itP;
  }

  cameraInfo.distortion_model = "plumb_bob";
  cameraInfo.d.resize(distortion.cols);
  const double *itD = distortion.ptr<double>(0, 0);
  for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
  {
    cameraInfo.d[i] = *itD;
  }
}

void Kinect2Bridge::callbackStatus()
{
  bool isSubscribedDepth = false;
  bool isSubscribedColor = false;

  lockStatus.lock();
  clientConnected = updateStatus(isSubscribedColor, isSubscribedDepth);
  bool error = false;

  if(clientConnected && !deviceActive)
  {
    OUT_INFO(node.get_logger(), "client connected. starting device...");
    if(!device->startStreams(isSubscribedColor, isSubscribedDepth))
    {
      OUT_ERROR(node.get_logger(), "could not start device!");
      error = true;
    }
    else
    {
      deviceActive = true;
    }
  }
  else if(!clientConnected && deviceActive)
  {
    OUT_INFO(node.get_logger(), "no clients connected. stopping device...");
    if(!device->stop())
    {
      OUT_ERROR(node.get_logger(), "could not stop device!");
      error = true;
    }
    else
    {
      deviceActive = false;
    }
  }
  else if(deviceActive && (isSubscribedColor != this->isSubscribedColor || isSubscribedDepth != this->isSubscribedDepth))
  {
    if(!device->stop())
    {
      OUT_ERROR(node.get_logger(), "could not stop device!");
      error = true;
    }
    else if(!device->startStreams(isSubscribedColor, isSubscribedDepth))
    {
      OUT_ERROR(node.get_logger(), "could not start device!");
      error = true;
      deviceActive = false;
    }
  }
  this->isSubscribedColor = isSubscribedColor;
  this->isSubscribedDepth = isSubscribedDepth;
  lockStatus.unlock();

  if(error)
  {
    stop();
  }
}

bool Kinect2Bridge::updateStatus(bool &isSubscribedColor, bool &isSubscribedDepth)
{
  isSubscribedDepth = false;
  isSubscribedColor = false;

  for(size_t i = 0; i < COUNT; ++i)
  {
    Status s = UNSUBCRIBED;
    if(imagePubs[i]->get_subscription_count() > 0)
    {
      s = RAW;
    }
    if(compressedPubs[i]->get_subscription_count() > 0)
    {
      s = s == RAW ? BOTH : COMPRESSED;
    }

    if(i <= COLOR_SD_RECT && s != UNSUBCRIBED)
    {
      isSubscribedDepth = true;
    }
    if(i >= COLOR_SD_RECT && s != UNSUBCRIBED)
    {
      isSubscribedColor = true;
    }

    status[i] = s;
  }
  if(infoHDPub->get_subscription_count() > 0 || infoQHDPub->get_subscription_count() > 0)
  {
    isSubscribedColor = true;
  }
  if(infoIRPub->get_subscription_count() > 0)
  {
    isSubscribedDepth = true;
  }

  return isSubscribedColor || isSubscribedDepth;
}

void Kinect2Bridge::setupMainThread()
{
  nextFrame = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::seconds>(deltaT);
  fpsTime = std::chrono::steady_clock::now();
  nextColor = true;
  nextIrDepth = true;
}

void Kinect2Bridge::main()
{
  if(!deviceActive)
  {
    fpsTime =  std::chrono::steady_clock::now() + 10ms;
    nextFrame = fpsTime + deltaT;
    return;
  }

  auto now = std::chrono::steady_clock::now();

  if(now - fpsTime >= 3.0s)
    fpsTime = now;

  if(now >= nextFrame)
  {
    nextColor = true;
    nextIrDepth = true;
    nextFrame += deltaT;
  }

  if(!deviceActive)
  {
    oldFrameIrDepth = frameIrDepth;
    oldFrameColor = frameColor;
    lockTime.lock();
    elapsedTimeColor = 0s;
    elapsedTimeIrDepth = 0s;
    lockTime.unlock();
  }
}

void Kinect2Bridge::threadDispatcher(const size_t id)
{
  setThreadName("Worker" + std::to_string(id));
  const size_t checkFirst = id % 2;
  bool processedFrame = false;
  int oldNice = nice(0);
  oldNice = nice(19 - oldNice);

  for(; running && rclcpp::ok();)
  {
    processedFrame = false;

    for(size_t i = 0; i < 2; ++i)
    {
      if(i == checkFirst)
      {
        if(nextIrDepth && lockIrDepth.try_lock())
        {
          nextIrDepth = false;
          receiveIrDepth();
          processedFrame = true;
        }
      }
      else
      {
        if(nextColor && lockColor.try_lock())
        {
          nextColor = false;
          receiveColor();
          processedFrame = true;
        }
      }
    }

    if(!processedFrame)
    {
      std::this_thread::sleep_for(1ms);
    }
  }
}

void Kinect2Bridge::receiveIrDepth()
{
  libfreenect2::FrameMap frames;
  cv::Mat depth, ir;
  std_msgs::msg::Header header;
  std::vector<cv::Mat> images(COUNT);
  std::vector<Status> status = this->status;
  size_t frame;

  if(!receiveFrames(listenerIrDepth, frames))
  {
    lockIrDepth.unlock();
    return;
  }
  auto now = std::chrono::steady_clock::now();

  header = createHeader(lastDepth, lastColor);

  libfreenect2::Frame *irFrame = frames[libfreenect2::Frame::Ir];
  libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];

  if(irFrame->status != 0 || depthFrame->status != 0)
  {
    listenerIrDepth->release(frames);
    lockIrDepth.unlock();
    running = false;
    OUT_ERROR(node.get_logger(), "failure in depth packet processor from libfreenect2");
    return;
  }
  if(irFrame->format != libfreenect2::Frame::Float || depthFrame->format != libfreenect2::Frame::Float)
  {
    listenerIrDepth->release(frames);
    lockIrDepth.unlock();
    running = false;
    OUT_ERROR(node.get_logger(), "received invalid frame format");
    return;
  }

  frame = frameIrDepth++;

  if(status[COLOR_SD_RECT] || status[DEPTH_SD] || status[DEPTH_SD_RECT] || status[DEPTH_QHD] || status[DEPTH_HD])
  {
    cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).copyTo(depth);
  }

  if(status[IR_SD] || status[IR_SD_RECT])
  {
    ir = cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data);
    ir.convertTo(images[IR_SD], CV_16U);
  }

  listenerIrDepth->release(frames);
  lockIrDepth.unlock();

  processIrDepth(depth, images, status);

  publishImages(images, header, status, frame, pubFrameIrDepth, IR_SD, COLOR_HD);

  auto elapsed = std::chrono::steady_clock::now() - now;
  lockTime.lock();
  elapsedTimeIrDepth += elapsed;
  lockTime.unlock();
}

void Kinect2Bridge::receiveColor()
{
  libfreenect2::FrameMap frames;
  std_msgs::msg::Header header;
  std::vector<cv::Mat> images(COUNT);
  std::vector<Status> status = this->status;
  size_t frame;

  if(!receiveFrames(listenerColor, frames))
  {
    lockColor.unlock();
    return;
  }
  auto now = std::chrono::steady_clock::now();

  header = createHeader(lastColor, lastDepth);

  libfreenect2::Frame *colorFrame = frames[libfreenect2::Frame::Color];

  if(colorFrame->status != 0)
  {
    listenerColor->release(frames);
    lockIrDepth.unlock();
    running = false;
    OUT_ERROR(node.get_logger(), "failure in rgb packet processor from libfreenect2");
    return;
  }
  if(colorFrame->format != libfreenect2::Frame::BGRX && colorFrame->format != libfreenect2::Frame::RGBX)
  {
    listenerColor->release(frames);
    lockIrDepth.unlock();
    running = false;
    OUT_ERROR(node.get_logger(), "received invalid frame format");
    return;
  }

  frame = frameColor++;

  cv::Mat color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC4, colorFrame->data);
  if(status[COLOR_SD_RECT])
  {
    lockRegSD.lock();
    memcpy(this->color.data, colorFrame->data, sizeColor.width * sizeColor.height * 4);
    this->color.format = colorFrame->format;
    lockRegSD.unlock();
  }
  if(status[COLOR_HD] || status[COLOR_HD_RECT] || status[COLOR_QHD] || status[COLOR_QHD_RECT] ||
     status[MONO_HD] || status[MONO_HD_RECT] || status[MONO_QHD] || status[MONO_QHD_RECT])
  {
    cv::Mat tmp;
    cv::flip(color, tmp, 1);
    if(colorFrame->format == libfreenect2::Frame::BGRX)
    {
      cv::cvtColor(tmp, images[COLOR_HD], cv::COLOR_BGRA2BGR);
    }
    else
    {
      cv::cvtColor(tmp, images[COLOR_HD], cv::COLOR_RGBA2BGR);
    }
  }

  listenerColor->release(frames);
  lockColor.unlock();

  processColor(images, status);

  publishImages(images, header, status, frame, pubFrameColor, COLOR_HD, COUNT);

  auto elapsed = std::chrono::steady_clock::now() - now;
  lockTime.lock();
  elapsedTimeColor += elapsed;
  lockTime.unlock();
}

bool Kinect2Bridge::receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames)
{
  bool newFrames = false;
  for(; !newFrames;)
  {
#ifdef LIBFREENECT2_THREADING_STDLIB
    newFrames = listener->waitForNewFrame(frames, 1000);
#else
    newFrames = true;
    listener->waitForNewFrame(frames);
#endif
    if(!deviceActive || !running || !rclcpp::ok())
    {
      if(newFrames)
      {
        listener->release(frames);
      }
      return false;
    }
  }
  return newFrames;
}

std_msgs::msg::Header Kinect2Bridge::createHeader(std::chrono::steady_clock::time_point &last, std::chrono::steady_clock::time_point &other)
{
  auto timestamp = std::chrono::steady_clock::now();
  lockSync.lock();
  if(other == std::chrono::steady_clock::time_point{})
  {
    last = timestamp;
  }
  else
  {
    timestamp = other;
    other = std::chrono::steady_clock::time_point{};
  }
  lockSync.unlock();

  std_msgs::msg::Header header;
  header.stamp = kinect2_bridge::TimeUtils::ToRosTimeMsg(timestamp);
  return header;
}

void Kinect2Bridge::processIrDepth(const cv::Mat &depth, std::vector<cv::Mat> &images, const std::vector<Status> &status)
{
  // COLOR registered to depth
  if(status[COLOR_SD_RECT])
  {
    cv::Mat tmp;
    libfreenect2::Frame depthFrame(sizeIr.width, sizeIr.height, 4, depth.data);
    libfreenect2::Frame undistorted(sizeIr.width, sizeIr.height, 4);
    libfreenect2::Frame registered(sizeIr.width, sizeIr.height, 4);
    lockRegSD.lock();
    registration->apply(&color, &depthFrame, &undistorted, &registered);
    lockRegSD.unlock();
    cv::flip(cv::Mat(sizeIr, CV_8UC4, registered.data), tmp, 1);
    if(color.format == libfreenect2::Frame::BGRX)
    {
      cv::cvtColor(tmp, images[COLOR_SD_RECT], cv::COLOR_BGRA2BGR);
    }
    else
    {
      cv::cvtColor(tmp, images[COLOR_SD_RECT], cv::COLOR_RGBA2BGR);
    }
  }

  // IR
  if(status[IR_SD] || status[IR_SD_RECT])
  {
    cv::flip(images[IR_SD], images[IR_SD], 1);
  }
  if(status[IR_SD_RECT])
  {
    cv::remap(images[IR_SD], images[IR_SD_RECT], map1Ir, map2Ir, cv::INTER_AREA);
  }

  // DEPTH
  cv::Mat depthShifted;
  if(status[DEPTH_SD])
  {
    depth.convertTo(images[DEPTH_SD], CV_16U, 1);
    cv::flip(images[DEPTH_SD], images[DEPTH_SD], 1);
  }
  if(status[DEPTH_SD_RECT] || status[DEPTH_QHD] || status[DEPTH_HD])
  {
    double _depthShift = 0.0;
    depth.convertTo(depthShifted, CV_16U, 1, _depthShift);
    depthShift = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(depthShift));
    cv::flip(depthShifted, depthShifted, 1);
  }
  if(status[DEPTH_SD_RECT])
  {
    cv::remap(depthShifted, images[DEPTH_SD_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
  }
  if(status[DEPTH_QHD])
  {
    lockRegLowRes.lock();
    depthRegLowRes->registerDepth(depthShifted, images[DEPTH_QHD]);
    lockRegLowRes.unlock();
  }
  if(status[DEPTH_HD])
  {
    lockRegHighRes.lock();
    depthRegHighRes->registerDepth(depthShifted, images[DEPTH_HD]);
    lockRegHighRes.unlock();
  }
}

void Kinect2Bridge::processColor(std::vector<cv::Mat> &images, const std::vector<Status> &status)
{
  // COLOR
  if(status[COLOR_HD_RECT] || status[MONO_HD_RECT])
  {
    cv::remap(images[COLOR_HD], images[COLOR_HD_RECT], map1Color, map2Color, cv::INTER_AREA);
  }
  if(status[COLOR_QHD] || status[MONO_QHD])
  {
    cv::resize(images[COLOR_HD], images[COLOR_QHD], sizeLowRes, 0, 0, cv::INTER_AREA);
  }
  if(status[COLOR_QHD_RECT] || status[MONO_QHD_RECT])
  {
    cv::remap(images[COLOR_HD], images[COLOR_QHD_RECT], map1LowRes, map2LowRes, cv::INTER_AREA);
  }

  // MONO
  if(status[MONO_HD])
  {
    cv::cvtColor(images[COLOR_HD], images[MONO_HD], cv::COLOR_BGR2GRAY);
  }
  if(status[MONO_HD_RECT])
  {
    cv::cvtColor(images[COLOR_HD_RECT], images[MONO_HD_RECT], cv::COLOR_BGR2GRAY);
  }
  if(status[MONO_QHD])
  {
    cv::cvtColor(images[COLOR_QHD], images[MONO_QHD], cv::COLOR_BGR2GRAY);
  }
  if(status[MONO_QHD_RECT])
  {
    cv::cvtColor(images[COLOR_QHD_RECT], images[MONO_QHD_RECT], cv::COLOR_BGR2GRAY);
  }
}

void Kinect2Bridge::publishImages(const std::vector<cv::Mat> &images, const std_msgs::msg::Header &header, const std::vector<Status> &status, const size_t frame, size_t &pubFrame, const size_t begin, const size_t end)
{
  std::vector<std::unique_ptr<sensor_msgs::msg::Image>> imageMsgs(COUNT);
  std::vector<std::unique_ptr<sensor_msgs::msg::CompressedImage>> compressedMsgs(COUNT);
  std::unique_ptr<sensor_msgs::msg::CameraInfo> infoHDMsg,  infoQHDMsg,  infoIRMsg;
  std_msgs::msg::Header _header = header;

  if(begin < COLOR_HD)
  {
    _header.frame_id = baseNameTF + K2_TF_IR_OPT_FRAME;

    infoIRMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    *infoIRMsg = infoIR;
    infoIRMsg->header = _header;
  }
  else
  {
    _header.frame_id = baseNameTF + K2_TF_RGB_OPT_FRAME;

    infoHDMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    *infoHDMsg = infoHD;
    infoHDMsg->header = _header;

    infoQHDMsg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    *infoQHDMsg = infoQHD;
    infoQHDMsg->header = _header;

  }

  for(size_t i = begin; i < end; ++i)
  {
    if(i < DEPTH_HD || i == COLOR_SD_RECT)
    {
      _header.frame_id = baseNameTF + K2_TF_IR_OPT_FRAME;
    }
    else
    {
      _header.frame_id = baseNameTF + K2_TF_RGB_OPT_FRAME;
    }

    switch(status[i])
    {
    case UNSUBCRIBED:
      break;
    case RAW:
      imageMsgs[i] = std::make_unique<sensor_msgs::msg::Image>();
      createImage(images[i], _header, Image(i), *imageMsgs[i]);
      break;
    case COMPRESSED:
      compressedMsgs[i] = std::make_unique<sensor_msgs::msg::CompressedImage>();
      createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
      break;
    case BOTH:
      imageMsgs[i] = std::make_unique<sensor_msgs::msg::Image>();
      compressedMsgs[i] = std::make_unique<sensor_msgs::msg::CompressedImage>();
      createImage(images[i], _header, Image(i), *imageMsgs[i]);
      createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
      break;
    }
  }

  while(frame != pubFrame)
  {
    std::this_thread::sleep_for(100us);
  }
  lockPub.lock();
  for(size_t i = begin; i < end; ++i)
  {
    switch(status[i])
    {
    case UNSUBCRIBED:
      break;
    case RAW:
      imagePubs[i]->publish(std::move(imageMsgs[i]));
      break;
    case COMPRESSED:
      compressedPubs[i]->publish(std::move(compressedMsgs[i]));
      break;
    case BOTH:
      imagePubs[i]->publish(std::move(imageMsgs[i]));
      compressedPubs[i]->publish(std::move(compressedMsgs[i]));
      break;
    }
  }

  if(begin < COLOR_HD)
  {
    if(infoIRPub->get_subscription_count() > 0)
    {
      infoIRPub->publish(std::move(infoIRMsg));
    }
  }
  else
  {
    if(infoHDPub->get_subscription_count() > 0)
    {
      infoHDPub->publish(std::move(infoHDMsg));
    }
    if(infoQHDPub->get_subscription_count() > 0)
    {
      infoQHDPub->publish(std::move(infoQHDMsg));
    }
  }

  ++pubFrame;
  lockPub.unlock();
}

void Kinect2Bridge::createImage(const cv::Mat &image, const std_msgs::msg::Header &header, const Image type, sensor_msgs::msg::Image &msgImage) const
{
  size_t step, size;
  step = image.cols * image.elemSize();
  size = image.rows * step;

  switch(type)
  {
  case IR_SD:
  case IR_SD_RECT:
  case DEPTH_SD:
  case DEPTH_SD_RECT:
  case DEPTH_HD:
  case DEPTH_QHD:
    msgImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    break;
  case COLOR_SD_RECT:
  case COLOR_HD:
  case COLOR_HD_RECT:
  case COLOR_QHD:
  case COLOR_QHD_RECT:
    msgImage.encoding = sensor_msgs::image_encodings::BGR8;
    break;
  case MONO_HD:
  case MONO_HD_RECT:
  case MONO_QHD:
  case MONO_QHD_RECT:
    msgImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    break;
  case COUNT:
    return;
  }

  msgImage.header = header;
  msgImage.height = image.rows;
  msgImage.width = image.cols;
  msgImage.is_bigendian = false;
  msgImage.step = step;
  msgImage.data.resize(size);
  memcpy(msgImage.data.data(), image.data, size);
}

void Kinect2Bridge::createCompressed(const cv::Mat &image, const std_msgs::msg::Header &header, const Image type, sensor_msgs::msg::CompressedImage &msgImage) const
{
  msgImage.header = header;

  switch(type)
  {
  case IR_SD:
  case IR_SD_RECT:
  case DEPTH_SD:
  case DEPTH_SD_RECT:
  case DEPTH_HD:
  case DEPTH_QHD:
    msgImage.format = compression16BitString;
    cv::imencode(compression16BitExt, image, msgImage.data, compressionParams);
    break;
  case COLOR_SD_RECT:
  case COLOR_HD:
  case COLOR_HD_RECT:
  case COLOR_QHD:
  case COLOR_QHD_RECT:
    msgImage.format = std::string(sensor_msgs::image_encodings::BGR8) + "; jpeg compressed bgr8";
    cv::imencode(".jpg", image, msgImage.data, compressionParams);
    break;
  case MONO_HD:
  case MONO_HD_RECT:
  case MONO_QHD:
  case MONO_QHD_RECT:
    msgImage.format = std::string(sensor_msgs::image_encodings::TYPE_8UC1) + "; jpeg compressed ";
    cv::imencode(".jpg", image, msgImage.data, compressionParams);
    break;
  case COUNT:
    return;
  }
}

void Kinect2Bridge::publishStaticTF()
{
  setThreadName("TFPublisher");
  /* TODO
  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform stColorOpt, stIrOpt;
  ros::Time now = std::chrono::steady_clock::now();

  tf::Matrix3x3 rot(rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
                    rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
                    rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));

  tf::Quaternion qZero;
  qZero.setRPY(0, 0, 0);
  tf::Vector3 trans(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));
  tf::Vector3 vZero(0, 0, 0);
  tf::Transform tIr(rot, trans), tZero(qZero, vZero);

  stColorOpt = tf::StampedTransform(tZero, now, baseNameTF + K2_TF_LINK, baseNameTF + K2_TF_RGB_OPT_FRAME);
  stIrOpt = tf::StampedTransform(tIr, now, baseNameTF + K2_TF_RGB_OPT_FRAME, baseNameTF + K2_TF_IR_OPT_FRAME);

  for(; running && rclcpp::ok();)
  {
    now = std::chrono::steady_clock::now();
    stColorOpt.stamp_ = now;
    stIrOpt.stamp_ = now;

    broadcaster.sendTransform(stColorOpt);
    broadcaster.sendTransform(stIrOpt);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  */
}

void Kinect2Bridge::setThreadName(const std::string &name)
{
#if defined(__linux__)
  prctl(PR_SET_NAME, name.c_str());
#elif defined(__APPLE__)
  pthread_setname_np(name.c_str());
#endif
}
