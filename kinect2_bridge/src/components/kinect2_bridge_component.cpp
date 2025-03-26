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

#include "bridge/kinect2_bridge.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>

namespace
{
constexpr const char* ROS_COMPONENT_NAME = "kinect2_bridge_component";
}

class Kinect2BridgeComponent : public rclcpp::Node
{
public:
  explicit Kinect2BridgeComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node(ROS_COMPONENT_NAME, options)
  {
    kinect2Bridge = std::make_unique<Kinect2Bridge>(*this);
    if (!kinect2Bridge->start())
      throw std::runtime_error("Failed to start kinect2_bridge");
  }

  ~Kinect2BridgeComponent() override
  {
    if (kinect2Bridge)
    {
      kinect2Bridge->stop();
      kinect2Bridge.reset();
    }
  }

private:
  std::unique_ptr<Kinect2Bridge> kinect2Bridge;
};

// Register the component with ROS 2
RCLCPP_COMPONENTS_REGISTER_NODE(Kinect2BridgeComponent)
