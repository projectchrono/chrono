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
// Authors: 肖言 (Yan Xiao), Shuo He
// =============================================================================
//
// Wraps data received from a flatbuffer SPAT message into a corresponding C++
// class
// See also flatbuffer/fbs/SPAT.fbs
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"

namespace chrono {
namespace synchrono {

SynSPATMessageState::SynSPATMessageState(const SynFlatBuffers::SPAT::State* state) {
    time = state->time();

    for (auto lane : *state->lanes())
        lanes.emplace_back(lane->intersection(), lane->approach(), lane->lane(), static_cast<LaneColor>(lane->color()));
}

SynSPATMessage::SynSPATMessage(int rank, std::shared_ptr<SynSPATMessageState> state)
    : SynMessage(rank, SynMessageType::SPAT) {
    m_state = state ? state : chrono_types::make_shared<SynSPATMessageState>();
}

void SynSPATMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_SPAT_State)
        return;

    const SynFlatBuffers::SPAT::State* state = message->message_as_SPAT_State();

    std::vector<IntersectionLane> lanes;
    for (auto lane : (*state->lanes())) {
        lanes.emplace_back(lane->intersection(), lane->approach(), lane->lane(), static_cast<LaneColor>(lane->color()));
    }

    m_state = chrono_types::make_shared<SynSPATMessageState>(state->time(), lanes);
}

FlatBufferMessage SynSPATMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    std::vector<flatbuffers::Offset<SynFlatBuffers::SPAT::Lane>> lanes;
    for (IntersectionLane lane : m_state->lanes) {
        auto color = SynFlatBuffers::SPAT::Color(lane.color);
        lanes.push_back(SynFlatBuffers::SPAT::CreateLane(builder, lane.intersection, lane.approach, lane.lane,
                                                         static_cast<SynFlatBuffers::SPAT::Color>(color)));
    }

    flatbuffers::Offset<SynFlatBuffers::SPAT::State> flatbuffer_state =
        SynFlatBuffers::SPAT::CreateState(builder, m_state->time, builder.CreateVector(lanes));

    FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
        SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_SPAT_State, flatbuffer_state.Union(), m_rank));
    return message;
}

void SynSPATMessage::setColor(int intersection, int approach, int lane_number, LaneColor color) {
    bool found = false;
    for (auto& lane : m_state->lanes) {
        if (lane.intersection == intersection && lane.approach == approach && lane.lane == lane_number) {
            found = true;
            lane.color = color;
        }
    }

    if (!found)
        m_state->lanes.push_back({intersection, approach, lane_number, color});
}

}  // namespace synchrono
}  // namespace chrono
