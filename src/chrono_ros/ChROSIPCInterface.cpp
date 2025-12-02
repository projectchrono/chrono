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
    std::cout << "Cleaning up IPC interface and subprocess..." << std::endl;
    if (m_subprocess_manager) {
        // Send shutdown message and terminate subprocess
        ipc::Message shutdown_msg;
        shutdown_msg.header = ipc::MessageHeader(ipc::MessageType::SHUTDOWN, 0, 0, m_sequence_counter++);
        m_subprocess_manager->SendMessage(shutdown_msg);
        
        // Give subprocess time to clean up
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "IPC cleanup complete" << std::endl;
}

void ChROSIPCInterface::Initialize(rclcpp::NodeOptions options) {
    std::cout << "Initializing IPC-based ROS interface for node: " << GetNodeName() << std::endl;
    
    // CRITICAL: Do NOT call base class Initialize() - it would initialize ROS in main process
    // where VSG symbols cause conflicts. All ROS initialization happens in the subprocess.
    
    // Launch the subprocess
    if (!m_subprocess_manager->LaunchSubprocess()) {
        throw std::runtime_error("Failed to launch ROS subprocess");
    }
    
    std::cout << "ROS subprocess launched successfully" << std::endl;
    
    // Wait a moment for subprocess to initialize
    std::cout << "Waiting for subprocess to fully initialize..." << std::endl;
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
        std::cerr << "ROS subprocess has terminated unexpectedly" << std::endl;
        return;
    }
    
    // Process any response messages from subprocess if needed
    ipc::Message response;
    while (m_subprocess_manager->ReceiveMessage(response)) {
        // Handle any response messages if needed in the future
    }
}

bool ChROSIPCInterface::SendHandlerData(ipc::MessageType message_type, const void* data, size_t size) {
    if (!m_subprocess_manager->IsSubprocessRunning()) {
        std::cout << "ERROR: Subprocess not running" << std::endl;
        return false;
    }
    
    // Create IPC message
    uint64_t timestamp_ns = static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    ipc::Message message(message_type, timestamp_ns, m_sequence_counter++, data, size);
    
    std::cout << "Attempting to send IPC message type " << static_cast<int>(message_type) 
              << " with size " << size << std::endl;
    
    bool result = m_subprocess_manager->SendMessage(message);
    std::cout << "Send result: " << (result ? "SUCCESS" : "FAILED") << std::endl;
    return result;
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