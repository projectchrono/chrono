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
// Manager for the ROS handlers
//
// =============================================================================

#include "chrono_ros/ChROSManager.h"

#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSIPCInterface.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"  
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"

#include "chrono/core/ChTypes.h"

namespace chrono {
namespace ros {

ChROSManager::ChROSManager(const std::string& node_name) {
    // Always use IPC interface to keep it clean and avoid ROS symbol issues
    m_interface = chrono_types::make_shared<ChROSIPCInterface>(node_name);
    std::cout << "ChROSManager: Using IPC interface" << std::endl;
}

void ChROSManager::Initialize() {
    // Initialize the interface (launches subprocess in IPC mode)
    m_interface->Initialize();

    // In IPC mode, skip handler initialization in main process
    // Handlers are only initialized in the subprocess where ROS is safe to use
    std::cout << "ChROSManager: IPC mode - handler initialization skipped in main process" << std::endl;
}

bool ChROSManager::Update(double time, double step) {
    static auto last_print = std::chrono::high_resolution_clock::now();
    static int update_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // IPC mode: Generic handler data collection and transmission
    // This approach works for ANY handler that implements GetSerializedData()
    auto ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(m_interface);
    if (ipc_interface) {
        // OUTGOING: Send handler data to subprocess
        for (auto handler : m_handlers) {
            auto handler_start = std::chrono::high_resolution_clock::now();
            
            // Get serialized data from handler (no ROS symbols, safe!)
            // Handler manages its own throttling via update rate
            auto data = handler->GetSerializedData(time);
            
            auto serialize_end = std::chrono::high_resolution_clock::now();
            
            if (!data.empty()) {
                // Determine message type based on handler type
                ipc::MessageType msg_type = GetHandlerMessageType(handler);
                
                // Send via IPC to subprocess - if buffer full, silently drop frame
                // This is standard practice for real-time sensor data
                bool sent = ipc_interface->SendHandlerData(msg_type, data.data(), data.size());
                
                auto send_end = std::chrono::high_resolution_clock::now();
                auto serialize_us = std::chrono::duration_cast<std::chrono::microseconds>(serialize_end - handler_start).count();
                auto send_us = std::chrono::duration_cast<std::chrono::microseconds>(send_end - serialize_end).count();
                
                if (serialize_us > 1000 || send_us > 1000) {
                    std::cout << "[TIMING] Handler serialize=" << serialize_us << "us, send=" << send_us 
                              << "us, sent=" << sent << ", size=" << data.size() << std::endl;
                }
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        
        update_count++;
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count();
        if (elapsed >= 2) {
            std::cout << "[PERF] Update() avg=" << (total_us) << "us, rate=" << (update_count / elapsed) << " Hz" << std::endl;
            last_print = now;
            update_count = 0;
        }
        
        // INCOMING: Check for messages from subprocess (bidirectional subscribers)
        // Reuse message buffer to avoid 64MB allocation per call
        static thread_local ipc::Message incoming_msg;
        while (ipc_interface->ReceiveMessage(incoming_msg)) {
            std::cout << "[MAIN PROCESS] ✓ Received IPC message from subprocess, type=" 
                      << static_cast<int>(incoming_msg.header.type) << std::endl;
            
            // Dispatch to handler using virtual method
            bool handled = false;
            for (auto handler : m_handlers) {
                if (handler->SupportsIncomingMessages()) {
                    // Check if this handler handles this message type
                    ipc::MessageType handler_type = GetHandlerMessageType(handler);
                    if (handler_type == incoming_msg.header.type) {
                        handler->HandleIncomingMessage(incoming_msg);
                        handled = true;
                        break;
                    }
                }
            }
            
            if (!handled) {
                std::cout << "[MAIN PROCESS] No handler for incoming message type " 
                          << static_cast<int>(incoming_msg.header.type) << std::endl;
            }
        }
    }

    m_interface->SpinSome();
    return true;  // Assume OK - subprocess handles ROS state
}

void ChROSManager::RegisterHandler(std::shared_ptr<ChROSHandler> handler) {
    m_handlers.push_back(handler);
}

// Helper function to determine message type from handler type
// This is the ONE place where handler types are identified
// Adding a new handler just requires adding one line here
ipc::MessageType ChROSManager::GetHandlerMessageType(std::shared_ptr<ChROSHandler> handler) {
    // Identify handler type and return corresponding message type
    // This uses dynamic_cast but is clean and centralized
    
    if (std::dynamic_pointer_cast<ChROSClockHandler>(handler)) {
        return ipc::MessageType::CLOCK_DATA;
    }
    
    if (std::dynamic_pointer_cast<ChROSBodyHandler>(handler)) {
        return ipc::MessageType::BODY_DATA;
    }
    
    if (std::dynamic_pointer_cast<ChROSTFHandler>(handler)) {
        return ipc::MessageType::TF_DATA;
    }
    
    if (std::dynamic_pointer_cast<ChROSDriverInputsHandler>(handler)) {
        return ipc::MessageType::DRIVER_INPUTS;
    }
    
    if (std::dynamic_pointer_cast<ChROSViperDCMotorControlHandler>(handler)) {
        return ipc::MessageType::VIPER_DC_MOTOR_CONTROL;
    }
    
    if (std::dynamic_pointer_cast<ChROSCameraHandler>(handler)) {
        return ipc::MessageType::CAMERA_DATA;
    }
    
    // Add more handler types here as they're implemented
    
    return ipc::MessageType::CUSTOM_DATA;  // Default for unknown types
}

}  // namespace ros
}  // namespace chrono