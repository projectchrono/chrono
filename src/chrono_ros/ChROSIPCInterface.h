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
// IPC-based ChROSInterface that forwards to subprocess to avoid VSG conflicts
//
// =============================================================================

#ifndef CH_ROS_IPC_INTERFACE_H
#define CH_ROS_IPC_INTERFACE_H

#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/ipc/ChROSSubprocessManager.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include <string>
#include <memory>

namespace chrono {
namespace ros {

/// IPC-based interface that runs ROS in a separate process to avoid symbol conflicts
class CH_ROS_API ChROSIPCInterface : public ChROSInterface {
public:
    /// Constructor
    /// @param node_name Name for the ROS node in the subprocess
    ChROSIPCInterface(const std::string& node_name);
    
    /// Destructor - ensures subprocess cleanup
    ~ChROSIPCInterface();
    
    /// Initialize the subprocess-based ROS interface
    void Initialize(rclcpp::NodeOptions options = rclcpp::NodeOptions()) override;
    
    /// Process IPC messages (replaces SpinSome functionality)
    void SpinSome(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;
    
    /// Override GetNode to return nullptr (no node in main process)
    rclcpp::Node::SharedPtr GetNode() override { return nullptr; }
    
    /// Send handler data to the subprocess for publishing
    /// @param message_type Type of handler data
    /// @param data Serialized data to send
    /// @param size Size of data in bytes
    bool SendHandlerData(ipc::MessageType message_type, const void* data, size_t size);
    
    /// Receive message from subprocess (bidirectional communication)
    /// @param message Message buffer to fill
    /// @return true if message was received, false if no messages available
    bool ReceiveMessage(ipc::Message& message);
    
private:
    std::string m_channel_name;
    std::unique_ptr<ipc::SubprocessManager> m_subprocess_manager;
    uint32_t m_sequence_counter;
    ipc::Message m_send_buffer;  ///< Reusable buffer to avoid 64MB allocations
    
    /// Generate unique channel name for this instance
    std::string GenerateChannelName() const;
};

}  // namespace ros
}  // namespace chrono

#endif