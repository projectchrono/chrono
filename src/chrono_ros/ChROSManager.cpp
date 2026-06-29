// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSHandler.h"

#include <csignal>
#include <stdexcept>

namespace chrono {
namespace ros {

namespace {
// Help clean shutdown
volatile std::sig_atomic_t g_interrupt_requested = 0;
bool g_signal_handler_installed = false;
void HandleInterrupt(int /*signum*/) {
    g_interrupt_requested = 1;
}
}  // namespace

ChROSManager::ChROSManager(const std::string& node_name) : m_bridge(std::make_shared<ChROSBridge>(node_name)) {}

ChROSManager::~ChROSManager() {
    m_bridge->Shutdown();
}

void ChROSManager::SetChannelCapacity(size_t sim_to_node_bytes, size_t node_to_sim_bytes) {
    m_bridge->SetChannelCapacity(sim_to_node_bytes, node_to_sim_bytes);
}

void ChROSManager::Initialize() {
    if (m_initialized) {
        return;
    }
    m_bridge->Initialize();

    if (!g_signal_handler_installed) {
        g_signal_handler_installed = true;
        std::signal(SIGINT, HandleInterrupt);
        std::signal(SIGTERM, HandleInterrupt);
    }

    for (size_t i = 0; i < m_handlers.size(); i++) {
        if (!m_handlers[i]->Initialize(*m_bridge)) {
            throw std::runtime_error("Chrono::ROS: handler " + std::to_string(i) +
                                     " (registration order) failed to initialize");
        }
    }
    m_initialized = true;
}

bool ChROSManager::Update(double time, double step) {
    if (!m_initialized) {
        throw std::runtime_error("Chrono::ROS: Update() called before Initialize()");
    }
    if (g_interrupt_requested || !m_bridge->IsNodeAlive()) {
        return false;  // Ctrl-C / SIGTERM, or the node exited
    }

    m_bridge->SetSimTime(time);
    // Inbound first: commands received by ROS apply to this step's state
    // before handlers extract and publish.
    m_bridge->ProcessIncoming();
    for (auto& handler : m_handlers) {
        handler->Advance(time, step);
    }
    return true;
}

void ChROSManager::RegisterHandler(std::shared_ptr<ChROSHandler> handler) {
    if (!handler) {
        throw std::runtime_error("Chrono::ROS: RegisterHandler called with a null handler");
    }
    m_handlers.push_back(handler);
    if (m_initialized) {
        if (!handler->Initialize(*m_bridge)) {
            m_handlers.pop_back();
            throw std::runtime_error("Chrono::ROS: handler failed to initialize");
        }
    }
}

}  // namespace ros
}  // namespace chrono
