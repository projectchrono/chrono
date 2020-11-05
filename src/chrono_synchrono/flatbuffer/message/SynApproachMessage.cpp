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
// Authors: 肖言 (Yan Xiao), Jay Taves
// =============================================================================
//
// Class for Approach Messages, those contains data on approaches, which are a
// set of lanes going towards/away from an intersection in the same direction
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"

namespace chrono {
namespace synchrono {

ApproachLane::ApproachLane(const SynFlatBuffers::Approach::Lane* lane) {
    width = lane->width();

    for (auto point : *lane->controlPoints())
        controlPoints.emplace_back(point->x(), point->y(), point->z());
}

SynApproachMessageState::SynApproachMessageState(const SynFlatBuffers::Approach::State* approach) {
    time = approach->time();
    rank = approach->rank();

    for (auto lane : *approach->lanes())
        lanes.emplace_back(lane);
}

SynApproachMessage::SynApproachMessage(int rank, std::shared_ptr<SynApproachMessageState> state)
    : SynMessage(rank, SynMessageType::APPROACH) {
    m_state = state ? state : chrono_types::make_shared<SynApproachMessageState>();
}

void SynApproachMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_Approach_State)
        return;

    m_state = chrono_types::make_shared<SynApproachMessageState>(message->message_as_Approach_State());
}

FlatBufferMessage SynApproachMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    std::vector<flatbuffers::Offset<SynFlatBuffers::Approach::Lane>> lanes;

    for (auto l : m_state->lanes) {
        std::vector<flatbuffers::Offset<SynFlatBuffers::Vector>> controlPoints;
        for (auto points : l.controlPoints) {
            controlPoints.push_back(SynFlatBuffers::CreateVector(builder, points.x(), points.y(), points.z()));
        }

        lanes.push_back(SynFlatBuffers::Approach::CreateLane(builder, l.width, builder.CreateVector(controlPoints)));
    }

    flatbuffers::Offset<SynFlatBuffers::Approach::State> flatbuffer_state =
        SynFlatBuffers::Approach::CreateState(builder, m_state->time, m_state->rank, builder.CreateVector(lanes));

    FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
        SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Approach_State, flatbuffer_state.Union(), m_rank));
    return message;
}

}  // namespace synchrono
}  // namespace chrono