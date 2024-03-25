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

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

namespace chrono {
namespace ros {

builtin_interfaces::msg::Time ChROSHandlerUtilities::GetROSTimestamp(double elapsed_time_s) {
    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = static_cast<int32_t>(elapsed_time_s);
    timestamp.nanosec = static_cast<uint32_t>((elapsed_time_s - timestamp.sec) * 1e9);
    return timestamp;
}

double ChROSHandlerUtilities::GetChronoTime(const builtin_interfaces::msg::Time& time) {
    return time.sec + time.nanosec * 1e-9;
}

bool ChROSHandlerUtilities::CheckROSTopicName(std::shared_ptr<ChROSInterface> interface,
                                              const std::string& topic_name) {
    try {
        std::string full_topic_name = rclcpp::expand_topic_or_service_name(topic_name, interface->GetNode()->get_name(),
                                                                           interface->GetNode()->get_namespace());
    } catch (rclcpp::exceptions::InvalidTopicNameError& e) {
        std::cerr << "ERROR: Topic '" << topic_name << "' is not a valid ROS topic name: " << e.what() << std::endl;
        return false;
    }

    return true;
}

}  // namespace ros
}  // namespace chrono