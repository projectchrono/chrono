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
// Authors: Yan Xiao, Jay Taves
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

SynApproachMessage::SynApproachMessage(unsigned int source_id, unsigned int destination_id)
    : SynMessage(source_id, destination_id) {}

void SynApproachMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_Approach_State)
        return;

    ConvertSPATFromFlatBuffers(message->message_as_Approach_State());
}

void SynApproachMessage::ConvertSPATFromFlatBuffers(const SynFlatBuffers::Approach::State* approach) {
    this->time = approach->time();

    for (auto lane : *approach->lanes())
        lanes.emplace_back(lane);
}

FlatBufferMessage SynApproachMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    std::vector<flatbuffers::Offset<SynFlatBuffers::Approach::Lane>> flatbuffer_lanes;
    flatbuffer_lanes.reserve(this->lanes.size());
    for (const auto& l : this->lanes) {
        std::vector<flatbuffers::Offset<SynFlatBuffers::Vector>> controlPoints;
        controlPoints.reserve(l.controlPoints.size());
        for (const auto& points : l.controlPoints) {
            controlPoints.push_back(SynFlatBuffers::CreateVector(builder, points.x(), points.y(), points.z()));
        }
        flatbuffer_lanes.push_back(
            SynFlatBuffers::Approach::CreateLane(builder, l.width, builder.CreateVector(controlPoints)));
    }

    flatbuffers::Offset<SynFlatBuffers::Approach::State> flatbuffer_state =
        SynFlatBuffers::Approach::CreateState(builder, this->time, builder.CreateVector(flatbuffer_lanes));

    FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(SynFlatBuffers::CreateMessage(
        builder, SynFlatBuffers::Type_Approach_State, flatbuffer_state.Union(), m_source_id, m_destination_id));
    return message;
}

}  // namespace synchrono
}  // namespace chrono