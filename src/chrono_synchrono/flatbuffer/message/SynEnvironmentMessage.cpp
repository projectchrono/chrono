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
// Authors: 肖言 (Yan Xiao)
// =============================================================================
//
// Message class for Environment Agents. This class is only used to send the
// initial zombie description for an Environment agent. Environment agents do
// not synchronize their state in any way so don't need messages for any other
// purpose.
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

#include "chrono/core/ChLog.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;
namespace Environment = SynFlatBuffers::Agent::Environment;

SynEnvironmentMessage::SynEnvironmentMessage(int rank, std::shared_ptr<SynEnvironmentMessageState> state)
    : SynAgentMessage(rank, SynMessageType::ENVIRONMENT) {
    m_state = state ? state : chrono_types::make_shared<SynEnvironmentMessageState>();
}

void SynEnvironmentMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    GetLog() << "Warning: SynEnvironmentMessage::StateFromMessage called"
             << "\n";
    // Environment messages aren't sent, so we don't set any state based on them
}

FlatBufferMessage SynEnvironmentMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    GetLog() << "Warning: SynEnvironmentMessage::MessageFromState called"
             << "\n";
    // Environment messages aren't sent, so we don't create any message based on them
    return SynFlatBuffers::CreateMessage(builder);
}

void SynEnvironmentMessage::DescriptionFromMessage(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;

    auto description = message->message_as_Agent_Description();
    m_rank = message->rank();
}

FlatBufferMessage SynEnvironmentMessage::MessageFromDescription(flatbuffers::FlatBufferBuilder& builder) {
    // No json initialization happens for EnvironmentAgents
    auto flatbuffer_json = builder.CreateString("");
    auto flatbuffer_type = Agent::Type_Environment_Description;

    auto env_description = Environment::CreateDescription(builder);
    auto flatbuffer_description =
        Agent::CreateDescription(builder, flatbuffer_type, env_description.Union(), flatbuffer_json);

    return SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Agent_Description,
                                         flatbuffer_description.Union(), m_rank);
}

}  // namespace synchrono
}  // namespace chrono