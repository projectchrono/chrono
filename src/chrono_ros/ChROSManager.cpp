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
// Manager for the ROS handlers
//
// =============================================================================

#include "chrono_ros/ChROSManager.h"

#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSIPCInterface.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include "chrono/core/ChTypes.h"
#include <iostream>

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

    std::cout << "ChROSManager: IPC interface initialized" << std::endl;
}

bool ChROSManager::Update(double time, double step) {
    // IPC mode: Generic handler data collection and transmission
    // This approach works for ANY handler that implements GetSerializedData()
    auto ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(m_interface);
    if (ipc_interface) {
        // OUTGOING: Send handler data to subprocess
        for (auto handler : m_handlers) {
            auto data = handler->GetSerializedData(time);
            if (!data.empty()) {
                ipc_interface->SendHandlerData(handler->GetMessageType(), data.data(), data.size());
            }
        }
        static thread_local ipc::Message incoming_msg;
        while (ipc_interface->ReceiveMessage(incoming_msg)) {
            // Dispatch to handler using virtual method
            bool handled = false;
            for (auto handler : m_handlers) {
                if (handler->SupportsIncomingMessages() && handler->GetMessageType() == incoming_msg.header.type) {
                    handler->HandleIncomingMessage(incoming_msg);
                    handled = true;
                    break;
                }
            }
            
            if (!handled) {
                std::cerr << "ChROSManager: No handler for incoming message type "
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

}  // namespace ros
}  // namespace chrono