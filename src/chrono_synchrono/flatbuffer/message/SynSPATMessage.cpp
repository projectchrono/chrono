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
// Authors: Yan Xiao, Shuo He
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

SynSPATMessage::SynSPATMessage(unsigned int source_id, unsigned int destination_id)
    : SynMessage(source_id, destination_id) {}

void SynSPATMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_SPAT_State)
        return;

    const SynFlatBuffers::SPAT::State* state = message->message_as_SPAT_State();

    this->lanes.clear();
    for (auto lane : (*state->lanes())) {
        this->lanes.emplace_back(lane->intersection(), lane->approach(), lane->lane(),
                                 static_cast<LaneColor>(lane->color()));
    }

    this->time = state->time();
}

void SynSPATMessage::ConvertSPATFromFlatBuffers(const SynFlatBuffers::SPAT::State* state) {
    this->time = state->time();

    for (auto lane : *state->lanes())
        this->lanes.emplace_back(lane->intersection(), lane->approach(), lane->lane(),
                                 static_cast<LaneColor>(lane->color()));
}

FlatBufferMessage SynSPATMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    std::vector<flatbuffers::Offset<SynFlatBuffers::SPAT::Lane>> flatbuffer_lanes;
    flatbuffer_lanes.reserve(this->lanes.size());
    for (const auto& lane : this->lanes) {
        auto color = SynFlatBuffers::SPAT::Color(lane.color);
        flatbuffer_lanes.push_back(SynFlatBuffers::SPAT::CreateLane(
            builder, lane.intersection, lane.approach, lane.lane, static_cast<SynFlatBuffers::SPAT::Color>(color)));
    }

    flatbuffers::Offset<SynFlatBuffers::SPAT::State> flatbuffer_state =
        SynFlatBuffers::SPAT::CreateState(builder, this->time, builder.CreateVector(flatbuffer_lanes));

    FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(SynFlatBuffers::CreateMessage(
        builder, SynFlatBuffers::Type_SPAT_State, flatbuffer_state.Union(), m_source_id, m_destination_id));
    return message;
}

void SynSPATMessage::SetColor(int intersection, int approach, int lane_number, LaneColor color) {
    bool found = false;
    for (auto& lane : this->lanes) {
        if (lane.intersection == intersection && lane.approach == approach && lane.lane == lane_number) {
            found = true;
            lane.color = color;
        }
    }

    if (!found)
        this->lanes.push_back({intersection, approach, lane_number, color});
}

}  // namespace synchrono
}  // namespace chrono
