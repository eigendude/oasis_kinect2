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

#include "kinect2_downscaler.h"

#include "kinect2_bridge/kinect2_definitions.h"

#include <algorithm>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

namespace
{
constexpr unsigned int DOWNSCALED_maxWidth = 640;
constexpr unsigned int DOWNSCALED_maxHeight = 480;
}

Kinect2Downscaler::Kinect2Downscaler(rclcpp::Node& node)
  : node(node)
{
  this->node.declare_parameter<std::string>("base_name", K2_DEFAULT_NS);
}

bool Kinect2Downscaler::start()
{
  std::string base_name;

  node.get_parameter_or("base_name", base_name, std::string(K2_DEFAULT_NS));

  publisher = image_transport::create_publisher(&node, base_name + K2_TOPIC_SD + K2_TOPIC_IMAGE_COLOR);
  subscriber = image_transport::create_subscription(&node, base_name + K2_TOPIC_QHD + K2_TOPIC_IMAGE_COLOR,
    [this](const auto& msg)
    {
      OnImage(msg);
    },
    "raw");

  return true;
}

void Kinect2Downscaler::stop()
{
  subscriber.shutdown();
  publisher.shutdown();
}

void Kinect2Downscaler::OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  // Convert the incoming image to an OpenCV image (assume bgr8 encoding)
  cv_bridge::CvImageConstPtr imagePtr;
  try
  {
    imagePtr = cv_bridge::toCvShare(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(node.get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat inputImage = imagePtr->image;

  const unsigned int maxWidth = DOWNSCALED_maxWidth;
  const unsigned int maxHeight = DOWNSCALED_maxHeight;

  // Compute scale factor to preserve aspect ratio
  double scaleWidth = static_cast<double>(maxWidth) / inputImage.cols;
  double scaleHeight = static_cast<double>(maxHeight) / inputImage.rows;

  double scale = std::min(scaleWidth, scaleHeight);

  // Ensure we only downscale if necessary (never upscale)
  scale = std::min(scale, 1.0);

  // Compute the target size
  cv::Size targetSize(static_cast<unsigned int>(inputImage.cols * scale),
                      static_cast<unsigned int>(inputImage.rows * scale));

  // Downscale the image using OpenCV
  cv::Mat resizedImage;
  cv::resize(inputImage, resizedImage, targetSize);

  // Convert the resized cv::Mat back into a ROS image message
  auto outMsg = cv_bridge::CvImage(msg->header, msg->encoding, resizedImage).toImageMsg();

  // Publish the downscaled image
  publisher.publish(*outMsg);
}
