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
// Store the Map information in the simulation. Currently only used for traffic
// light. A Map -> Intersections -> approaches -> lanes
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"

namespace chrono {
namespace synchrono {

Intersection::Intersection(const SynFlatBuffers::MAP::intersection* intersection) {
    for (auto approach : *intersection->approaches())
        approaches.emplace_back(approach);
}

SynMAPMessageState::SynMAPMessageState(const SynFlatBuffers::MAP::State* state) {
    time = state->time();
    rank = state->rank();

    for (auto intersection : *state->intersections())
        intersections.push_back(Intersection(intersection));
}

SynMAPMessage::SynMAPMessage(int rank, std::shared_ptr<SynMAPMessageState> state)
    : SynMessage(rank, SynMessageType::MAP) {
    m_state = state ? state : chrono_types::make_shared<SynMAPMessageState>();
}

void SynMAPMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_MAP_State)
        return;

    SynMAPMessageState state(message->message_as_MAP_State());
    m_state->time = state.time;
    m_state->intersections = state.intersections;
    m_state->rank = state.rank;
}

FlatBufferMessage SynMAPMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    std::vector<flatbuffers::Offset<SynFlatBuffers::MAP::intersection>> intersections;
    for (auto intersection : m_state->intersections) {
        std::vector<flatbuffers::Offset<SynFlatBuffers::Approach::State>> Approaches;
        for (auto approach : intersection.approaches) {
            std::vector<flatbuffers::Offset<SynFlatBuffers::Approach::Lane>> lanes;
            for (auto lane : approach.lanes) {
                std::vector<flatbuffers::Offset<SynFlatBuffers::Vector>> control_points;

                for (auto point : lane.controlPoints) {
                    control_points.push_back(SynFlatBuffers::CreateVector(builder, point.x(), point.y(), point.z()));
                }
                lanes.push_back(SynFlatBuffers::Approach::CreateLaneDirect(builder, lane.width, &control_points));
            }

            Approaches.push_back(
                SynFlatBuffers::Approach::CreateStateDirect(builder, approach.time, approach.rank, &lanes));
        }
        intersections.push_back(SynFlatBuffers::MAP::Createintersection(builder, builder.CreateVector(Approaches)));
    }

    flatbuffers::Offset<SynFlatBuffers::MAP::State> flatbuffer_state =
        SynFlatBuffers::MAP::CreateState(builder, m_state->time, m_state->rank, builder.CreateVector(intersections));

    FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
        SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_MAP_State, flatbuffer_state.Union(), m_rank));
    return message;
}

int SynMAPMessage::AddLane(int intersection, int approach, ApproachLane lane) {
    // Adding in blank intersections/approaches until we fill up to the intersection/approach you actually wanted to add
    while (m_state->intersections.size() <= intersection)
        m_state->intersections.emplace_back();

    while (m_state->intersections[intersection].approaches.size() <= approach)
        m_state->intersections[intersection].approaches.emplace_back();

    int num_lanes = m_state->intersections[intersection].approaches[approach].lanes.size();
    m_state->intersections[intersection].approaches[approach].lanes.push_back(lane);

    return num_lanes;
}

}  // namespace synchrono
}  // namespace chrono