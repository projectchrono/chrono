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
// Authors: Patrick Chen
// =============================================================================
//
// Subprocess (ROS node) publisher for crane state data.
// Compiled ONLY into the chrono_ros_node executable.
//
// Receives CraneStateData via IPC, creates a Float64MultiArray [s, sd],
// and publishes it on the topic specified in the IPC payload.
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/mbs/ChROSHydraulicCraneHandler_ipc.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "std_msgs/msg/float64_multi_array.hpp"

#include <cstring>
#include <string>
#include <unordered_map>

namespace chrono {
namespace ros {

void PublishCraneStateToROS(const uint8_t* data,
                            size_t data_size,
                            rclcpp::Node::SharedPtr node,
                            ipc::IPCChannel* /*channel*/) {
    if (data_size < sizeof(ipc::CraneStateData))
        return;

    ipc::CraneStateData crane{};
    std::memcpy(&crane, data, sizeof(ipc::CraneStateData));

    // Lazy-create publisher keyed by topic name
    static std::unordered_map<std::string,
                              rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr>
        publishers;

    std::string topic(crane.topic_name);
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
        RCLCPP_INFO(node->get_logger(), "Created crane state publisher on %s", topic.c_str());
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "crane_state";
    msg.layout.dim[0].size = 2;
    msg.layout.dim[0].stride = 2;
    msg.data = {crane.s, crane.sd};
    publishers[topic]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(CRANE_STATE_DATA, PublishCraneStateToROS)

}  // namespace ros
}  // namespace chrono
