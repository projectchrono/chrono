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
// Clock publisher handler
//
// =============================================================================

#ifndef CH_ROS_CLOCK_HANDLER
#define CH_ROS_CLOCK_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "rosgraph_msgs/msg/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <string>
#include <cstring>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

// =============================================================================
// IPC DATA STRUCTURE (for subprocess communication)
// =============================================================================

/// Data structure for clock handler communication between processes.
/// This struct is serialized and sent via IPC from main process to subprocess.
/// Requirements:
/// - Plain C++ types only (no STL, no pointers, no ROS types)
/// - Fixed size for reliable serialization
/// - Trivially copyable (use memcpy)
struct ChROSClockData {
    double time_seconds;  ///< Simulation time in seconds
};

// =============================================================================
// HANDLER CLASS (main process)
// =============================================================================

/// Publishes rosgraph_msgs::msg::Clock messages to synchronize ROS time with simulation time.
///
/// PUBLISHER PATTERN:
/// - Main process: Calls GetSerializedData() each tick → sends IPC
/// - Subprocess: Receives IPC → calls registered function → publishes ROS message
///
/// Implementation files:
/// - ChROSClockHandler.cpp: Main process logic (this handler, Tick not used in IPC mode)
/// - ChROSClockHandler_ros.cpp: Subprocess ROS publishing (compiled into chrono_ros_node)
///
/// To implement a similar publisher handler:
/// 1. Define IPC struct above (plain C++ types)
/// 2. Implement GetSerializedData() to pack your data
/// 3. Create YourHandler_ros.cpp with publishing function
/// 4. Register with CHRONO_ROS_REGISTER_HANDLER(YOUR_MESSAGE_TYPE, YourPublishFunction)
/// 5. Add YOUR_MESSAGE_TYPE to MessageType enum in ChROSIPCMessage.h
/// 6. Add handler recognition to ChROSManager::GetHandlerMessageType()
class CH_ROS_API ChROSClockHandler : public ChROSHandler {
  public:
    ChROSClockHandler(double update_rate = 0, const std::string& topic_name = "/clock");

    /// Initialize handler (called once at startup in main process)
    /// In IPC mode, this does nothing since ROS node is in subprocess
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    
    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::CLOCK_DATA; }

    /// Extract simulation time for IPC transmission to subprocess
    /// Called each tick in main process. Packs ChROSClockData struct into byte vector.
    /// Subprocess receives this data and publishes ROS message.
    /// @param time Current simulation time in seconds
    /// @return Serialized ChROSClockData as byte vector
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    const std::string m_topic_name;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_publisher;
};

///@} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif