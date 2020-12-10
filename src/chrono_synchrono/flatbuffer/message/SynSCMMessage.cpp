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
// Deformable terrain. See chrono_vehicle/terrain/SCMDeformableTerrain.* for
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
SynSCMMessage::SynSCMMessage(int rank, std::shared_ptr<SynSCMTerrainState> state)
    : SynMessage(rank, SynMessageType::SCM_TERRAIN) {
    m_state = state ? state : chrono_types::make_shared<SynSCMTerrainState>();
}

void SynSCMMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Terrain::SCM::State
    if (message->message_type() != SynFlatBuffers::Type_Terrain_State)
        return;

    auto terrain_state = message->message_as_Terrain_State();
    auto state = terrain_state->message_as_SCM_State();

    std::vector<SCMDeformableTerrain::NodeLevel> modified_nodes;
    auto nodes_size = state->nodes()->size();
    for (int i = 0; i < nodes_size; i++) {
        auto fb_node = state->nodes()->Get(i);

        auto node = std::make_pair(ChVector2<>(fb_node->x(), fb_node->y()), fb_node->level());
        modified_nodes.push_back(node);
    }

    m_state->time = state->time();
    m_state->modified_nodes = modified_nodes;
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynSCMMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    std::vector<SCM::NodeLevel> modified_nodes(m_state->modified_nodes.size());
    for (auto& node : m_state->modified_nodes)
        modified_nodes.push_back(SCM::NodeLevel(node.first.x(), node.first.y(), node.second));

    auto scm_state = SCM::CreateStateDirect(builder, m_state->time, &modified_nodes);

    auto flatbuffer_state = Terrain::CreateState(builder, Terrain::Type::Type_SCM_State, scm_state.Union());
    auto flatbuffer_message = SynFlatBuffers::CreateMessage(builder,                             //
                                                            SynFlatBuffers::Type_Terrain_State,  //
                                                            flatbuffer_state.Union(),            //
                                                            m_rank);                             //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

}  // namespace synchrono
}  // namespace chrono
