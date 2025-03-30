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

#include "time_utils.h"

using namespace kinect2_bridge;

builtin_interfaces::msg::Time TimeUtils::ToRosTimeMsg(std::chrono::time_point<std::chrono::steady_clock> timestamp)
{
  builtin_interfaces::msg::Time msg;

  const std::chrono::steady_clock::time_point epoch{};
  uint64_t elapsedTimeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - epoch).count();
  msg.sec = static_cast<uint32_t>(elapsedTimeNs / 1e9);
  msg.nanosec = static_cast<uint32_t>(elapsedTimeNs - (msg.sec * 1e9));

  return msg;
}
