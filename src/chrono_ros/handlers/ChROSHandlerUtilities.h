// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
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

/// Utility class with static functions that may be useful for ROS handlers
class CH_ROS_API ChROSHandlerUtilities {
  public:
    /// Constructs a builtin_interfaces::msg::Time (helpful for std_msgs::msg::Header::stamp) msg from the current
    /// simulation time
    /// @param elapsed_time_s the current elapsed simulation time in seconds
    static builtin_interfaces::msg::Time GetROSTimestamp(double elapsed_time_s);

    /// Converts a builtin_interfaces::msg::Time to a chrono time
    /// @param time the builtin_interfaces::msg::Time to convert
    static double GetChronoTime(const builtin_interfaces::msg::Time& time);

    /// Constructs a relative topic name given variable number of inputs. A relative topic name, when used to construct
    /// a ROS entity (publisher, subscription, etc.), is resolved to map to the node namespace. Using a relative topic
    /// name is recommended as then all topics attached to the node are prepended with a like namespace. Usage:
    /// BuildRelativeTopicName("sensing", "camera", "front", "rgb") constructs "~/sensing/camera/front/rgb"
    template <typename... Args>
    static std::string BuildRelativeTopicName(Args&&... args) {
        std::stringstream ss;
        ss << "~";
        ((ss << '/' << args), ...);
        return ss.str();
    }

    /// Checks the passed topic name is resolvable. Refer to the ROS documentation for information regarding what
    /// defines a qualified topic name.
    static bool CheckROSTopicName(std::shared_ptr<ChROSInterface> interface, const std::string& topic_name);
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif