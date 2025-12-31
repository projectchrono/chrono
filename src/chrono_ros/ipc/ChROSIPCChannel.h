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
// IPC channel for bidirectional communication using shared memory ring buffers
//
// =============================================================================

#ifndef CH_ROS_IPC_CHANNEL_H
#define CH_ROS_IPC_CHANNEL_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/ipc/ChROSSharedMemory.h"
#include "chrono_ros/ipc/ChROSRingBuffer.h"
#include <memory>
#include <string>

namespace chrono {
namespace ros {
namespace ipc {

/// Bidirectional IPC channel using shared memory ring buffers
class CH_ROS_API IPCChannel {
public:
    /// Create IPC channel for the main process (creates shared memory)
    /// @param channel_name Unique name for this IPC channel
    /// @param buffer_size Size of each ring buffer in bytes (default: 512MB for high-bandwidth sensor data)
    ///                    Note: Total shared memory is 2x this size (bidirectional buffers)
    ///                    512MB allows ~145 full 1280x720 RGBA8 images or ~15 4K images per direction
    static std::unique_ptr<IPCChannel> CreateMainChannel(const std::string& channel_name, size_t buffer_size = 512 * 1024 * 1024);
    
    /// Connect to existing IPC channel from subprocess (opens existing shared memory)
    /// @param channel_name Name of the IPC channel to connect to
    static std::unique_ptr<IPCChannel> ConnectToChannel(const std::string& channel_name);
    
    /// Destructor
    ~IPCChannel();
    
    /// Send a message through the channel
    /// @param message Message to send
    /// @return true if successful, false if channel is full
    bool SendMessage(const Message& message);
    
    /// Receive a message from the channel (non-blocking)
    /// @param message Output message buffer
    /// @return true if message received, false if no message available
    bool ReceiveMessage(Message& message);
    
    /// Check if channel is ready for communication
    bool IsReady() const;
    
    /// Get the channel name
    const std::string& GetName() const { return m_name; }

private:
    /// Private constructor (use static factory methods)
    IPCChannel(const std::string& name, std::unique_ptr<SharedMemory> memory, bool is_main_process);
    
    /// Initialize ring buffers in shared memory
    void InitializeBuffers();
    
    std::string m_name;
    std::unique_ptr<SharedMemory> m_shared_memory;
    std::unique_ptr<RingBuffer> m_send_buffer;
    std::unique_ptr<RingBuffer> m_receive_buffer;
    bool m_is_main_process;
    uint32_t m_sequence_counter;
    
    // Shared memory layout
    struct SharedLayout {
        uint64_t buffer_size;  // Size of each ring buffer
        // Ring buffer for main -> subprocess communication
        alignas(64) char main_to_sub_buffer[];
        // Ring buffer for subprocess -> main communication  
        // (starts at offset buffer_size)
    };
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif