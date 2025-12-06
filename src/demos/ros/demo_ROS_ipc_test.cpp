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
// Simple test of the IPC-based ROS interface with clock handler
//
// =============================================================================

#include <iostream>
#include <chrono>
#include <thread>

#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSIPCInterface.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"

using namespace chrono;
using namespace chrono::ros;

/// Test ROS manager that uses IPC interface
class TestROSManager {
public:
    TestROSManager(const std::string& node_name) {
        m_interface = chrono_types::make_shared<ChROSIPCInterface>(node_name);
    }
    
    void Initialize() {
        m_interface->Initialize();
        
        // In IPC mode, we don't initialize handlers with ROS objects
        // They will be handled by the subprocess via IPC messages
        std::cout << "IPC interface initialized, handlers will communicate via subprocess" << std::endl;
    }
    
    bool Update(double time, double step) {
        // In IPC mode, we manually send clock data
        auto ipc_interface = std::dynamic_pointer_cast<ChROSIPCInterface>(m_interface);
        
        // Send clock data
        ipc::ClockData clock_data;
        clock_data.time_seconds = time;
        
        bool sent = ipc_interface->SendHandlerData(ipc::MessageType::CLOCK_DATA, &clock_data, sizeof(clock_data));
        if (sent) {
            std::cout << "Sent clock data: " << time << "s" << std::endl;
        } else {
            std::cout << "Failed to send clock data at " << time << "s" << std::endl;
        }
        
        m_interface->SpinSome();
        return true;
    }
    
    void RegisterHandler(std::shared_ptr<ChROSHandler> handler) {
        m_handlers.push_back(handler);
    }

private:
    std::shared_ptr<ChROSIPCInterface> m_interface;
    std::vector<std::shared_ptr<ChROSHandler>> m_handlers;
};

int main(int argc, char* argv[]) {
    std::cout << "Testing IPC-based ROS interface" << std::endl;

    // Create a simple system
    ChSystemNSC sys;

    // Create test ROS manager with IPC interface
    auto ros_manager = std::make_unique<TestROSManager>("test_ipc_node");

    // Create a clock handler (won't actually initialize ROS objects in test)
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Initialize ROS manager (launches subprocess)
    try {
        ros_manager->Initialize();
        std::cout << "ROS manager initialized successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize ROS manager: " << e.what() << std::endl;
        return 1;
    }

    // Simple simulation loop
    double time = 0.0;
    double step_size = 0.01;
    double end_time = 5.0;

    std::cout << "Starting simulation for " << end_time << " seconds" << std::endl;

    while (time < end_time) {
        // Update simulation time
        time += step_size;
        
        // Update ROS handlers via IPC
        if (!ros_manager->Update(time, step_size)) {
            std::cerr << "ROS update failed, terminating" << std::endl;
            break;
        }

        // Sleep to simulate real-time execution
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Test completed successfully" << std::endl;
    return 0;
}