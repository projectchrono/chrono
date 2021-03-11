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
// Authors: Yan Xiao
// =============================================================================
//
// Message class for Environment Agents. This class is only used to send the
// initial zombie description for an Environment agent. Environment agents do
// not synchronize their state in any way so don't need messages for any other
// purpose.
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;
namespace Environment = SynFlatBuffers::Agent::Environment;

SynEnvironmentMessage::SynEnvironmentMessage(unsigned int source_id, unsigned int destination_id)
    : SynMessage(source_id, destination_id) {
    map_message = chrono_types::make_shared<SynMAPMessage>(0, 0);
    spat_message = chrono_types::make_shared<SynSPATMessage>(0, 0);
}

void SynEnvironmentMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Agent::TrafficLight::State
    if (message->message_type() != SynFlatBuffers::Type_Agent_State)
        return;

    m_source_id = message->source_id();
    m_destination_id = message->destination_id();

    const SynFlatBuffers::Agent::State* agent_state = message->message_as_Agent_State();
    const SynFlatBuffers::Agent::Environment::State* state = agent_state->message_as_Environment_State();

    map_message->ConvertFromFlatBuffers(state->map());
    spat_message->ConvertFromFlatBuffers(state->spat());
}

FlatBufferMessage SynEnvironmentMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto map = map_message->ConvertToFlatBuffers(builder);
    auto spat = spat_message->ConvertToFlatBuffers(builder);

    auto environment_type = Agent::Type_Environment_State;
    auto environment_state = Environment::CreateState(builder, map, spat).Union();

    auto flatbuffer_state = Agent::CreateState(builder, environment_type, environment_state);

    auto flatbuffer_message = SynFlatBuffers::CreateMessage(builder,                           //
                                                            SynFlatBuffers::Type_Agent_State,  //
                                                            flatbuffer_state.Union(),          //
                                                            m_source_id, m_destination_id);    //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

}  // namespace synchrono
}  // namespace chrono