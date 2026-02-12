// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Utilities useful for all ROS handlers
//
// =============================================================================

#ifndef CH_ROS_HANDLER_UTILITIES_H
#define CH_ROS_HANDLER_UTILITIES_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSInterface.h"

#include "builtin_interfaces/msg/time.hpp"

#include <string>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

// =============================================================================
// UTILITY CLASS (no IPC needed)
// =============================================================================

/// Utility class with static helper functions for ROS handlers.
/// These functions are used by both main process and subprocess handlers.
/// No IPC support needed - these are pure utility functions.
class CH_ROS_API ChROSHandlerUtilities {
  public:
    /// Convert Chrono simulation time to ROS timestamp
    /// Used when creating ROS message headers
    /// @param elapsed_time_s Simulation time in seconds
    /// @return ROS timestamp message
    static builtin_interfaces::msg::Time GetROSTimestamp(double elapsed_time_s);

    /// Convert ROS timestamp to Chrono simulation time
    /// @param time ROS timestamp message
    /// @return Simulation time in seconds
    static double GetChronoTime(const builtin_interfaces::msg::Time& time);

    /// Construct a relative topic name from path components
    /// Relative topic names start with "~/" and are resolved to the node namespace
    /// Example: BuildRelativeTopicName("sensing", "camera", "rgb") → "~/sensing/camera/rgb"
    /// @param args Variable number of path components
    /// @return Relative topic name string
    template <typename... Args>
    static std::string BuildRelativeTopicName(Args&&... args) {
        std::stringstream ss;
        ss << "~";
        ((ss << '/' << args), ...);
        return ss.str();
    }

    /// Validate that a topic name is valid according to ROS naming rules
    /// @param interface ROS interface with node context
    /// @param topic_name Topic name to validate
    /// @return true if valid, false otherwise
    static bool CheckROSTopicName(std::shared_ptr<ChROSInterface> interface, const std::string& topic_name);
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif