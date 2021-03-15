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
// =============================================================================
//
// Class that handles communication across ranks or external entities. A
// communicator is something that passes messages over some protocol and
// interfaecs either a rank with another rank, a rank with an external process
// or really anything that relies on communication over some network interface.
//
// This class is implemented as a very generic abstract handler that holds and
// defines common functions and variables used by all communicators.
//
// =============================================================================

#include "chrono_synchrono/communication/SynCommunicator.h"
#include "chrono_synchrono/flatbuffer/message/SynSimulationMessage.h"

namespace chrono {
namespace synchrono {

SynCommunicator::SynCommunicator() : m_initialized(false) {}

SynCommunicator::~SynCommunicator() {}

void SynCommunicator::Initialize() {
    m_initialized = true;
}

// -----------------------------------------------------------------------------------------------

void SynCommunicator::Reset() {
    m_incoming_messages.clear();
}

void SynCommunicator::AddOutgoingMessages(SynMessageList& messages) {
    for (auto message : messages)
        m_flatbuffers_manager.AddMessage(message);
}

void SynCommunicator::AddQuitMessage() {
    // Source and destination are meaningless in this case
    auto message = chrono_types::make_shared<SynSimulationMessage>(AgentKey(), AgentKey(), true);
    m_flatbuffers_manager.AddMessage(message);
}

void SynCommunicator::AddIncomingMessages(SynMessageList& messages) {
    for (auto message : messages)
        m_incoming_messages.push_back(message);
}

void SynCommunicator::ProcessBuffer(std::vector<uint8_t>& data) {
    m_flatbuffers_manager.ProcessBuffer(data, m_incoming_messages);
}

// -----------------------------------------------------------------------------------------------

}  // namespace synchrono
}  // namespace chrono