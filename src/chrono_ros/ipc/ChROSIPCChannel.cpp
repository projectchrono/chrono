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

#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include <stdexcept>
#include <cstring>

namespace chrono {
namespace ros {
namespace ipc {

std::unique_ptr<IPCChannel> IPCChannel::CreateMainChannel(const std::string& channel_name, size_t buffer_size) {
    // Total size: two ring buffers
    size_t total_size = buffer_size * 2;
    
    auto memory = std::make_unique<SharedMemory>(channel_name, total_size, true);
    if (!memory->IsValid()) {
        throw std::runtime_error("Failed to create shared memory for IPC channel: " + channel_name);
    }
    
    return std::unique_ptr<IPCChannel>(new IPCChannel(channel_name, std::move(memory), true));
}

std::unique_ptr<IPCChannel> IPCChannel::ConnectToChannel(const std::string& channel_name) {
    auto memory = std::make_unique<SharedMemory>(channel_name, 0, false);
    if (!memory->IsValid()) {
        throw std::runtime_error("Failed to connect to shared memory for IPC channel: " + channel_name);
    }
    
    return std::unique_ptr<IPCChannel>(new IPCChannel(channel_name, std::move(memory), false));
}

IPCChannel::IPCChannel(const std::string& name, std::unique_ptr<SharedMemory> memory, bool is_main_process)
    : m_name(name), m_shared_memory(std::move(memory)), m_is_main_process(is_main_process), m_sequence_counter(0) {
    
    InitializeBuffers();
}

IPCChannel::~IPCChannel() = default;

void IPCChannel::InitializeBuffers() {
    char* base_ptr = static_cast<char*>(m_shared_memory->GetPtr());
    size_t total_size = m_shared_memory->GetSize();
    size_t buffer_size = total_size / 2;
    
    
    // Main process writes to first buffer, reads from second
    // Subprocess writes to second buffer, reads from first
    if (m_is_main_process) {
        m_send_buffer = std::make_unique<RingBuffer>(base_ptr, buffer_size);
        m_receive_buffer = std::make_unique<RingBuffer>(base_ptr + buffer_size, buffer_size);
    } else {
        m_send_buffer = std::make_unique<RingBuffer>(base_ptr + buffer_size, buffer_size);
        m_receive_buffer = std::make_unique<RingBuffer>(base_ptr, buffer_size);
    }
}

bool IPCChannel::SendMessage(const Message& message) {
    if (!IsReady()) {
        return false;
    }
    
    // Use atomic write for header + payload to prevent race conditions
    // The reader will see either the full message or nothing
    if (!m_send_buffer->Write(&message.header, sizeof(MessageHeader), 
                              message.payload.get(), message.header.payload_size)) {
        // Buffer full
        return false;
    }
    
    return true;
}

bool IPCChannel::ReceiveMessage(Message& message) {
    if (!IsReady()) {
        return false;
    }
    
    // First, try to read the header
    if (!m_receive_buffer->Read(&message.header, sizeof(MessageHeader))) {
        return false;  // No message available
    }
    
    // Validate header
    if (!message.header.IsValid()) {
        // Invalid header, skip to next potential message
        // In a more robust implementation, we'd try to resynchronize
        return false;
    }
    
    // Check payload size bounds
    if (message.header.payload_size > MAX_PAYLOAD_SIZE) {
        return false;  // Payload too large, likely corrupted
    }
    
    // Read payload if present
    if (message.header.payload_size > 0) {
        if (!m_receive_buffer->Read(message.payload.get(), message.header.payload_size)) {
            return false;  // Incomplete message
        }
    }
    
    return true;
}

bool IPCChannel::IsReady() const {
    return m_shared_memory && m_shared_memory->IsValid() && 
           m_send_buffer && m_receive_buffer;
}

}  // namespace ipc
}  // namespace ros
}  // namespace chrono