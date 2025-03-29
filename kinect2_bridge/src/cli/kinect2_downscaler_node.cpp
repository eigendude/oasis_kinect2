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

#include "image/kinect2_downscaler.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace
{
// Default node name
constexpr const char* ROS_NODE_NAME = "kinect2_downscaler";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(ROS_NODE_NAME);

  Kinect2Downscaler kinect2Downscaler{*node};
  if (!kinect2Downscaler.start())
  {
    RCLCPP_ERROR(node->get_logger(), "Error starting kinect2_downscaler");
    return 1;
  }

  rclcpp::spin(node);

  kinect2Downscaler.stop();

  rclcpp::shutdown();

  return 0;
}
