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
