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

#include <string>
#include <rclcpp/logging.hpp>
#include <rcutils/logging_macros.h>

// Set this to '0' to disable the extended colored output
#define EXTENDED_OUTPUT 1

#if EXTENDED_OUTPUT

#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"

const std::string getFunctionName(const std::string &name);
#define OUT_AUX(LOGGER, FUNC_COLOR, MSG_COLOR, STREAM, MSG) STREAM(LOGGER, FUNC_COLOR "[" << getFunctionName(__PRETTY_FUNCTION__) << "] " MSG_COLOR << MSG << NO_COLOR)

#define OUT_DEBUG(logger, msg) OUT_AUX(logger, FG_BLUE, NO_COLOR, RCLCPP_DEBUG_STREAM, msg)
#define OUT_INFO(logger, msg) OUT_AUX(logger, FG_GREEN, NO_COLOR, RCLCPP_INFO_STREAM, msg)
#define OUT_WARN(logger, msg) OUT_AUX(logger, FG_YELLOW, FG_YELLOW, RCLCPP_WARN_STREAM, msg)
#define OUT_ERROR(logger, msg) OUT_AUX(logger, FG_RED, FG_RED, RCLCPP_ERROR_STREAM, msg)

#else

#define NO_COLOR        ""
#define FG_BLACK        ""
#define FG_RED          ""
#define FG_GREEN        ""
#define FG_YELLOW       ""
#define FG_BLUE         ""
#define FG_MAGENTA      ""
#define FG_CYAN         ""

#define OUT_DEBUG(msg) ROS_DEBUG_STREAM(msg)
#define OUT_INFO(msg) ROS_INFO_STREAM(msg)
#define OUT_WARN(msg) ROS_WARN_STREAM(msg)
#define OUT_ERROR(msg) ROS_WARN_STREAM(msg)

#endif
