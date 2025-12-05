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

#include "chrono_ros/ChROSIPCInterface.h"
#include <iostream>
#include <sstream>
#include <random>
#include <thread>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

namespace chrono {
namespace ros {

ChROSIPCInterface::ChROSIPCInterface(const std::string& node_name) 
    : ChROSInterface(node_name), m_sequence_counter(0) {
    
    m_channel_name = GenerateChannelName();
    m_subprocess_manager = std::make_unique<ipc::SubprocessManager>(GetNodeName(), m_channel_name);
}

ChROSIPCInterface::~ChROSIPCInterface() {
    if (m_subprocess_manager) {
        // Send shutdown message and terminate subprocess
        ipc::Message shutdown_msg;
        shutdown_msg.header = ipc::MessageHeader(ipc::MessageType::SHUTDOWN, 0, 0, m_sequence_counter++);
        m_subprocess_manager->SendMessage(shutdown_msg);
        
        // Give subprocess time to clean up
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ChROSIPCInterface::Initialize(rclcpp::NodeOptions options) {
    // Launch the subprocess
    if (!m_subprocess_manager->LaunchSubprocess()) {
        throw std::runtime_error("Failed to launch ROS subprocess");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Increased delay
    
    // Verify subprocess is running
    if (!m_subprocess_manager->IsSubprocessRunning()) {
        throw std::runtime_error("ROS subprocess failed to start");
    }
    
    std::cout << "IPC interface initialization complete" << std::endl;
}

void ChROSIPCInterface::SpinSome(std::chrono::nanoseconds max_duration) {
    // Check if subprocess is still running
    if (!m_subprocess_manager->IsSubprocessRunning()) {
        // Only print once to avoid spamming
        static bool printed = false;
        if (!printed) {
            std::cerr << "ROS subprocess has terminated unexpectedly" << std::endl;
            printed = true;
        }
        return;
    }
    
    // Process any response messages from subprocess if needed
    // Use static buffer to avoid 64MB allocation every frame!
    static ipc::Message response;
    while (m_subprocess_manager->ReceiveMessage(response)) {
        // Handle any response messages if needed in the future
    }
}

bool ChROSIPCInterface::SendHandlerData(ipc::MessageType message_type, const void* data, size_t size) {
    if (!m_subprocess_manager->IsSubprocessRunning()) {
        return false;
    }
    
    // Reuse buffer, just update header and copy payload
    uint64_t timestamp_ns = static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    m_send_buffer.header = ipc::MessageHeader(message_type, timestamp_ns, static_cast<uint32_t>(size), m_sequence_counter++);
    
    if (size > 0 && data) {
        if (size > ipc::MAX_PAYLOAD_SIZE) {
            size = ipc::MAX_PAYLOAD_SIZE;
            m_send_buffer.header.payload_size = static_cast<uint32_t>(size);
        }
        std::memcpy(m_send_buffer.payload.get(), data, size);
    }
    
    return m_subprocess_manager->SendMessage(m_send_buffer);
}

bool ChROSIPCInterface::ReceiveMessage(ipc::Message& message) {
    return m_subprocess_manager->ReceiveMessage(message);
}

std::string ChROSIPCInterface::GenerateChannelName() const {
    std::stringstream ss;
    
#ifdef _WIN32
    ss << "chrono_ros_" << GetCurrentProcessId() << "_";
#else
    ss << "chrono_ros_" << getpid() << "_";
#endif
    
    // Add random component to ensure uniqueness
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1000, 9999);
    ss << dis(gen);
    
    return ss.str();
}

}  // namespace ros
}  // namespace chrono