/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kinect2_bridge/kinect2_bridge.h"
#include "rclcpp/rclcpp.hpp"

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
  }

  ~Kinect2BridgeComponent()
  {
    if (kinectBridge)
      kinectBridge->stop();
  }

  // Called after construction
  void on_init()
  {
    auto node_ptr = this->shared_from_this();
    kinectBridge = std::make_unique<Kinect2Bridge>(node_ptr);
    if (!kinectBridge->start()) {
      throw std::runtime_error("Failed to start kinect2_bridge");
    }
  }

private:
  std::unique_ptr<Kinect2Bridge> kinectBridge;
};

// Register the component with ROS 2
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Kinect2BridgeComponent)
