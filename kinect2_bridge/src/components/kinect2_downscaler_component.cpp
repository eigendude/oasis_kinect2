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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace
{
constexpr const char* ROS_COMPONENT_NAME = "kinect2_downscaler_component";
}

class Kinect2DownscalerComponent
{
public:
  Kinect2DownscalerComponent(const rclcpp::NodeOptions & options)
    : node(std::make_shared<rclcpp::Node>(ROS_COMPONENT_NAME, options))
  {
    interface = std::make_unique<Kinect2Downscaler>(*node);
    if (!interface->start())
      throw std::runtime_error(std::string{"Failed to start "} + node->get_name());
  }

  ~Kinect2DownscalerComponent()
  {
    if (interface)
    {
      interface->stop();
      interface.reset();
    }
  }

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface()
  {
    return node->get_node_base_interface();
  }

private:
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<Kinect2Downscaler> interface;
};

// Register the component with ROS 2
RCLCPP_COMPONENTS_REGISTER_NODE(Kinect2DownscalerComponent);
