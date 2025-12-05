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
// Handler registry for automatic IPC dispatch
// Handlers register their publishing functions here for subprocess use
//
// =============================================================================

#ifndef CH_ROS_HANDLER_REGISTRY_H
#define CH_ROS_HANDLER_REGISTRY_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <memory>

namespace chrono {
namespace ros {

/// Registry for handler publishing functions in the subprocess.
/// Each handler type registers a function that deserializes data and publishes to ROS.
/// This makes the subprocess dispatcher completely generic - it just looks up and calls
/// the registered function for each message type.
///
/// BIDIRECTIONAL: Handlers can also use the IPC channel to send data back to main process
/// (e.g., subscribers sending received ROS messages back to Chrono simulation)
class ChROSHandlerRegistry {
public:
    /// Function signature for handler publish functions
    /// Takes serialized data bytes (raw pointer + size to avoid copy), ROS node, and IPC channel
    using PublishFunction = std::function<void(const uint8_t*, size_t,
                                               rclcpp::Node::SharedPtr,
                                               ipc::IPCChannel*)>;
    
    /// Get the singleton instance
    static ChROSHandlerRegistry& GetInstance() {
        static ChROSHandlerRegistry instance;
        return instance;
    }
    
    /// Register a handler's publish function
    /// @param msg_type Message type identifier
    /// @param publish_func Function to call when this message type is received
    void RegisterPublisher(ipc::MessageType msg_type, PublishFunction publish_func) {
        m_publishers[msg_type] = publish_func;
    }
    
    /// Publish data using the registered handler
    /// @param msg_type Message type identifier
    /// @param data Pointer to serialized data bytes
    /// @param data_size Size of data in bytes
    /// @param node ROS node for publishing
    /// @param channel IPC channel for bidirectional communication (can be nullptr)
    /// @return true if handler was found and called, false otherwise
    bool Publish(ipc::MessageType msg_type, const uint8_t* data, size_t data_size,
                rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel = nullptr) {
        auto it = m_publishers.find(msg_type);
        if (it != m_publishers.end()) {
            it->second(data, data_size, node, channel);
            return true;
        }
        return false;
    }
    
    /// Check if a handler is registered for this message type
    bool HasPublisher(ipc::MessageType msg_type) const {
        return m_publishers.find(msg_type) != m_publishers.end();
    }
    
private:
    ChROSHandlerRegistry() = default;
    ~ChROSHandlerRegistry() = default;
    ChROSHandlerRegistry(const ChROSHandlerRegistry&) = delete;
    ChROSHandlerRegistry& operator=(const ChROSHandlerRegistry&) = delete;
    
    std::unordered_map<ipc::MessageType, PublishFunction> m_publishers;
};

/// Helper macro to register a handler's publish function
/// Place this in the handler's .cpp file (the part compiled in subprocess)
/// Usage: CHRONO_ROS_REGISTER_HANDLER(CLOCK_DATA, PublishClockToROS)
#define CHRONO_ROS_REGISTER_HANDLER(EnumValue, PublishFunc) \
    namespace { \
        struct HandlerRegistrar_##EnumValue { \
            HandlerRegistrar_##EnumValue() { \
                ::chrono::ros::ChROSHandlerRegistry::GetInstance().RegisterPublisher( \
                    ::chrono::ros::ipc::MessageType::EnumValue, PublishFunc); \
            } \
        }; \
        static HandlerRegistrar_##EnumValue g_registrar_##EnumValue; \
    }

}  // namespace ros
}  // namespace chrono

#endif