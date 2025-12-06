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
// Standalone ROS node process for chrono_ros IPC communication
// This process runs in isolation from VSG symbols to avoid conflicts
//
// Generic dispatcher using handler registry.
//
// =============================================================================

#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>

#ifndef _WIN32
    #include <sys/prctl.h>  // For PR_SET_PDEATHSIG
    #include <signal.h>
#endif

#include "rclcpp/rclcpp.hpp"

#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/ChROSHandlerRegistry.h"

using namespace chrono::ros::ipc;
using namespace chrono::ros;

/// Generic ROS node that receives handler data via IPC and publishes to ROS
/// Uses handler registry for completely extensible dispatch - no hard-coded handlers!
class ChronoROSNode {
public:
    ChronoROSNode(const std::string& node_name, const std::string& channel_name)
        : m_node_name(node_name), m_channel_name(channel_name) {
        
#ifndef _WIN32
        // CRITICAL: Set up parent death signal so subprocess dies if parent crashes
        // This prevents orphan ROS nodes from accumulating
        prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif
        
        // Initialize ROS
        rclcpp::init(0, nullptr);
        m_node = std::make_shared<rclcpp::Node>(node_name);
        m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        m_executor->add_node(m_node);
        
        // Connect to IPC channel
        RCLCPP_INFO(m_node->get_logger(), "Connecting to IPC channel: %s", channel_name.c_str());
        try {
            m_channel = IPCChannel::ConnectToChannel(channel_name);
            RCLCPP_INFO(m_node->get_logger(), "IPC channel connected successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to connect to IPC channel: %s", e.what());
            throw;
        }
        
        RCLCPP_INFO(m_node->get_logger(), "Chrono ROS node initialized: %s", node_name.c_str());
    }
    
    ~ChronoROSNode() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
    
    /// Main execution loop - completely generic, no handler-specific code!
    void Run() {
        // Allocate message buffer once and reuse to avoid 64MB allocations per loop
        Message message;
        RCLCPP_INFO(m_node->get_logger(), "Starting main execution loop");
        
        while (rclcpp::ok()) {
            // PRIORITY: Drain IPC queue FIRST before ROS callbacks
            // This prevents frames from piling up while ROS is busy
            int messages_processed = 0;
            while (m_channel->ReceiveMessage(message)) {
                messages_processed++;
                
                // Check for shutdown
                if (message.header.type == MessageType::SHUTDOWN) {
                    RCLCPP_INFO(m_node->get_logger(), "Received shutdown message");
                    return;
                }
                
                // Generic dispatch using registry - pass payload directly (no copy!)
                if (!ChROSHandlerRegistry::GetInstance().Publish(
                        message.header.type, 
                        message.payload.get(), 
                        message.header.payload_size,
                        m_node, 
                        m_channel.get())) {
                    RCLCPP_WARN(m_node->get_logger(), "No handler registered for message type: %d", 
                               static_cast<int>(message.header.type));
                }
            }
            
            // Process ROS callbacks AFTER draining IPC to keep pipeline flowing
            // Only spin if we didn't process many messages (avoid starving ROS)
            if (messages_processed < 10) {
                m_executor->spin_some(std::chrono::microseconds(100));
                // Very small delay to prevent busy waiting
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    }

private:
    std::string m_node_name;
    std::string m_channel_name;
    
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::unique_ptr<IPCChannel> m_channel;
};

int main(int argc, char* argv[]) {
    std::string node_name = "chrono_ros_node";
    std::string channel_name = "chrono_ros_ipc";

    // Parse command line arguments
    for (int i = 1; i < argc - 1; ++i) {
        std::string arg = argv[i];
        if (arg == "--node-name") {
            node_name = argv[++i];
        } else if (arg == "--channel-name") {
            channel_name = argv[++i];
        }
    }

    try {
        ChronoROSNode node(node_name, channel_name);
        node.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
