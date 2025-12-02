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

#ifdef CHRONO_VSG
    #include "chrono_ros/ChROSIPCInterface.h"
    #include "chrono_ros/ipc/ChROSIPCMessage.h"
    #include "chrono_ros/handlers/ChROSClockHandler.h"
    #include "chrono_ros/handlers/ChROSBodyHandler.h"  
    #include "chrono_ros/handlers/ChROSTFHandler.h"
#endif

#include "chrono/core/ChTypes.h"

namespace chrono {
namespace ros {

ChROSManager::ChROSManager(const std::string& node_name) {
#ifdef CHRONO_VSG
    // Use IPC interface when VSG module is available to avoid symbol collision
    m_interface = chrono_types::make_shared<ChROSIPCInterface>(node_name);
    std::cout << "ChROSManager: Using IPC interface (VSG module detected)" << std::endl;
#else
    // Use direct interface when VSG is not available
    m_interface = chrono_types::make_shared<ChROSInterface>(node_name);
    std::cout << "ChROSManager: Using direct interface (no VSG module)" << std::endl;
#endif
}

void ChROSManager::Initialize() {
    // Initialize the interface (launches subprocess in IPC mode)
    m_interface->Initialize();

#ifdef CHRONO_VSG
    // In IPC mode, completely skip handler initialization to avoid crashes
    // Handlers are kept for potential future data serialization
    std::cout << "ChROSManager: IPC mode - skipping all handler initialization" << std::endl;
#else
    // In direct mode, initialize handlers normally
    for (auto itr = m_handlers.begin(); itr != m_handlers.end();) {
        auto handler = *itr;
        if (!handler->Initialize(m_interface)) {
            std::cerr << "Failed to initialize ROS handler. Will remove handler and continue." << std::endl;
            itr = m_handlers.erase(itr);
        } else {
            itr++;
        }
    }
#endif
}

bool ChROSManager::Update(double time, double step) {
#ifdef CHRONO_VSG
    // In IPC mode, collect handler data and send via IPC
    auto ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(m_interface);
    if (ipc_interface) {
        // Always send clock data
        ipc::ClockData clock_data;
        clock_data.time_seconds = time;
        ipc_interface->SendHandlerData(ipc::MessageType::CLOCK_DATA, 
                                       &clock_data, sizeof(clock_data));
        
        // Process handlers for data collection and serialization
        for (auto handler : m_handlers) {
            if (handler->GetUpdateRate() == 0 || 
                (handler->GetUpdateRate() > 0 && fmod(time, 1.0/handler->GetUpdateRate()) < step/2.0)) {
                
                // Collect and serialize handler data without calling Initialize/Tick
                SerializeHandlerData(handler, time, ipc_interface);
            }
        }
    }
#else
    // Direct ROS mode - call handlers directly
    for (auto handler : m_handlers)
        handler->Update(time, step);
#endif

    m_interface->SpinSome();

#ifdef CHRONO_VSG
    return true;  // Assume OK - subprocess manager will detect failures
#else
    return rclcpp::ok();
#endif
}

void ChROSManager::RegisterHandler(std::shared_ptr<ChROSHandler> handler) {
    m_handlers.push_back(handler);
}

#ifdef CHRONO_VSG
void ChROSManager::SerializeHandlerData(std::shared_ptr<ChROSHandler> handler, double time,
                                       std::shared_ptr<ChROSIPCInterface> ipc_interface) {
    // Use dynamic_cast to identify handler types and extract their data
    
    // Try Clock Handler (already working)
    if (auto clock_handler = std::dynamic_pointer_cast<chrono::ros::ChROSClockHandler>(handler)) {
        // Clock data already handled in main Update loop
        return;
    }
    
    // Try Body Handler
    if (auto body_handler = std::dynamic_pointer_cast<chrono::ros::ChROSBodyHandler>(handler)) {
        SerializeBodyHandler(body_handler, time, ipc_interface);
        return;
    }
    
    // Try TF Handler
    if (auto tf_handler = std::dynamic_pointer_cast<chrono::ros::ChROSTFHandler>(handler)) {
        SerializeTFHandler(tf_handler, time, ipc_interface);
        return;
    }
    
    // Add more handler types as needed
    std::cout << "Unknown handler type - skipping serialization" << std::endl;
}

void ChROSManager::SerializeBodyHandler(std::shared_ptr<ChROSBodyHandler> handler, double time,
                                       std::shared_ptr<ChROSIPCInterface> ipc_interface) {
    auto body = handler->GetBody();
    if (!body) return;
    
    // Create body data structure  
    ipc::BodyData body_data;
    
    // Get body info
    const std::string& body_name = body->GetName();
    const std::string& topic = handler->GetTopic();
    
    strncpy(body_data.body_name, body_name.c_str(), sizeof(body_data.body_name) - 1);
    body_data.body_name[sizeof(body_data.body_name) - 1] = '\0';
    strncpy(body_data.topic_prefix, topic.c_str(), sizeof(body_data.topic_prefix) - 1);  
    body_data.topic_prefix[sizeof(body_data.topic_prefix) - 1] = '\0';
    
    // Get pose data
    auto pos = body->GetPos();
    auto rot = body->GetRot();
    body_data.pos_x = pos.x();
    body_data.pos_y = pos.y();
    body_data.pos_z = pos.z();
    body_data.rot_w = rot.e0();
    body_data.rot_x = rot.e1(); 
    body_data.rot_y = rot.e2();
    body_data.rot_z = rot.e3();
    
    // Get velocity data
    auto lin_vel = body->GetPosDt();
    auto ang_vel = body->GetAngVelParent();
    body_data.lin_vel_x = lin_vel.x();
    body_data.lin_vel_y = lin_vel.y();
    body_data.lin_vel_z = lin_vel.z();
    body_data.ang_vel_x = ang_vel.x();
    body_data.ang_vel_y = ang_vel.y();
    body_data.ang_vel_z = ang_vel.z();
    
    // Get acceleration data  
    auto lin_acc = body->GetPosDt2();
    auto ang_acc = body->GetAngAccParent();
    body_data.lin_acc_x = lin_acc.x();
    body_data.lin_acc_y = lin_acc.y();
    body_data.lin_acc_z = lin_acc.z();
    body_data.ang_acc_x = ang_acc.x();
    body_data.ang_acc_y = ang_acc.y();
    body_data.ang_acc_z = ang_acc.z();
    
    // Send via IPC
    ipc_interface->SendHandlerData(ipc::MessageType::BODY_DATA, &body_data, sizeof(body_data));
}

void ChROSManager::SerializeTFHandler(std::shared_ptr<ChROSTFHandler> handler, double time,
                                     std::shared_ptr<ChROSIPCInterface> ipc_interface) {
    // TODO: Implement TF handler serialization
    // For now, send minimal TF data
    
    ipc::TFData tf_data;
    tf_data.transform_count = 1; // Placeholder
    
    // Create a simple transform
    ipc::TFTransform transform;
    strncpy(transform.parent_frame, "world", sizeof(transform.parent_frame) - 1);
    strncpy(transform.child_frame, "base_link", sizeof(transform.child_frame) - 1);
    transform.pos_x = 0.0;
    transform.pos_y = 0.0; 
    transform.pos_z = 0.0;
    transform.rot_w = 1.0;
    transform.rot_x = 0.0;
    transform.rot_y = 0.0;
    transform.rot_z = 0.0;
    
    // For now, we need a better way to serialize variable-length TF data
    // Send placeholder data
    ipc_interface->SendHandlerData(ipc::MessageType::TF_DATA, &tf_data, sizeof(tf_data));
}
#endif

}  // namespace ros
}  // namespace chrono