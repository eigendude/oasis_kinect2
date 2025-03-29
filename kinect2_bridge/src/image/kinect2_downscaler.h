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

#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rclcpp
{
class Node;
}

class Kinect2Downscaler
{
public:
  Kinect2Downscaler(rclcpp::Node& node);

  bool start();
  void stop();

private:
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  // Construction parameters
  rclcpp::Node& node;

  // ROS parameters
  image_transport::Subscriber subscriber;
  image_transport::Publisher publisher;
};
