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

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>

namespace kinect2_bridge
{

class TimeUtils
{
public:
  static builtin_interfaces::msg::Time ToRosTimeMsg(std::chrono::time_point<std::chrono::steady_clock> timestamp);
};

}
