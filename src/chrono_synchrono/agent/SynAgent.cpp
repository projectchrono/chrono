// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Class that acts as the primary wrapper for an underlying Chrono object. This
// base class provides the functionality to synchronize between different agents
// on different ranks. Message generation and processing is done through a agent.
// Dynamic updates of the underlying Chrono objects are also performed by each
// agent on the object they actually wrap.
//
// =============================================================================

#include "chrono_synchrono/agent/SynAgent.h"

namespace chrono {
namespace synchrono {

SynAgent::SynAgent(AgentKey agent_key) : m_agent_key(agent_key) {}

SynAgent::~SynAgent() {}

void SynAgent::ProcessMessage(std::shared_ptr<SynMessage> msg) {
    if (m_process_message_callback)
        m_process_message_callback(msg);
}

// -------------------------------------------------------------------------

void SynAgent::SetProcessMessageCallback(std::function<void(std::shared_ptr<SynMessage>)> callback) {
    m_process_message_callback = callback;
}

}  // namespace synchrono
}  // namespace chrono