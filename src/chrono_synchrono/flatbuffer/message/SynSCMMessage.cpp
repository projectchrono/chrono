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
// Authors: Jay Taves
// =============================================================================
//
// Class that wraps data contained in a message about Soil Contact Model (SCM)
// Deformable terrain. See chrono_vehicle/terrain/SCMTerrain.* for
// more details.
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynSCMMessage.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

namespace Terrain = SynFlatBuffers::Terrain;
namespace SCM = SynFlatBuffers::Terrain::SCM;

/// Constructors
SynSCMMessage::SynSCMMessage(AgentKey source_key, AgentKey destination_key) : SynMessage(source_key, destination_key) {}

void SynSCMMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Terrain::SCM::State
    if (message->message_type() != SynFlatBuffers::Type_Terrain_State)
        return;

    m_source_key = AgentKey(message->source_key());
    m_destination_key = message->destination_key();

    auto terrain_state = message->message_as_Terrain_State();
    auto state = terrain_state->message_as_SCM_State();

    auto nodes_size = state->nodes()->size();
    modified_nodes.clear();
    modified_nodes.reserve(nodes_size);
    for (size_t i = 0; i < nodes_size; i++) {
        auto fb_node = state->nodes()->Get((flatbuffers::uoffset_t)i);
        auto node = std::make_pair(ChVector2<>(fb_node->x(), fb_node->y()), fb_node->level());
        modified_nodes.push_back(node);
    }

    this->time = state->time();
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynSCMMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    std::vector<SCM::NodeLevel> modified_nodes;
    modified_nodes.reserve(this->modified_nodes.size());
    for (const auto& node : this->modified_nodes)
        modified_nodes.push_back(SCM::NodeLevel(node.first.x(), node.first.y(), node.second));

    auto scm_state = SCM::CreateStateDirect(builder, time, &modified_nodes);

    auto flatbuffer_state = Terrain::CreateState(builder, Terrain::Type::Type_SCM_State, scm_state.Union());
    auto flatbuffer_message =
        SynFlatBuffers::CreateMessage(builder,                                                                   //
                                      SynFlatBuffers::Type_Terrain_State,                                        //
                                      flatbuffer_state.Union(),                                                  //
                                      m_source_key.GetFlatbuffersKey(), m_destination_key.GetFlatbuffersKey());  //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

}  // namespace synchrono
}  // namespace chrono
