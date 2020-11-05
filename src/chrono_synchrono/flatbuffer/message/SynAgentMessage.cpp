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
// Class handling the conversion from agent flatbuffer messages into C++ objects
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;

SynAgentMessage::SynAgentMessage(int rank, SynMessageType type, std::string json) : SynMessage(rank, type) {
    m_description = chrono_types::make_shared<SynAgentDescription>(json);
}

// ---------------------------------------------------------------------------------------------------------------

void SynAgentMessage::StateFromMessage(const SynFlatBuffers::Message* message) {}

// ---------------------------------------------------------------------------------------------------------------

/// Generate agent description from FlatBuffers message
void SynAgentMessage::DescriptionFromMessage(const SynFlatBuffers::Message* message) {
    /// Cast from SynFlatBuffers::Message to SynFlatBuffers::Agent::Description
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;
    auto description = message->message_as_Agent_Description();

    m_description->json = description->json()->str();
}

}  // namespace synchrono
}  // namespace chrono
