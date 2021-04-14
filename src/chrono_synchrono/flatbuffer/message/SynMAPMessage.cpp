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
// Store the Map information in the simulation. Currently only used for traffic
// light. A Map -> Intersections -> approaches -> lanes
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"

namespace chrono {
namespace synchrono {

SynMAPMessage::SynMAPMessage(unsigned int source_id, unsigned int destination_id)
    : SynMessage(source_id, destination_id) {}

void SynMAPMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_MAP_State)
        return;

    m_source_id = message->source_id();
    m_destination_id = message->destination_id();

    auto state = message->message_as_MAP_State();
    this->time = state->time();

    for (auto flatbuffer_intersection : *state->intersections()) {
        Intersection intersection;
        for (auto flatbuffer_approach : *flatbuffer_intersection->approaches()) {
            auto approach = chrono_types::make_shared<SynApproachMessage>(m_source_id, m_destination_id);
            approach->ConvertSPATFromFlatBuffers(flatbuffer_approach);
            intersection.approaches.push_back(approach);
        }
        this->intersections.push_back(intersection);
    }
}

FlatBufferMessage SynMAPMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    std::vector<flatbuffers::Offset<SynFlatBuffers::MAP::intersection>> flatbuffer_intersections;
    flatbuffer_intersections.reserve(intersections.size());
    for (const auto& intersection : intersections) {
        std::vector<flatbuffers::Offset<SynFlatBuffers::Approach::State>> flatbuffer_approaches;
        flatbuffer_approaches.reserve(intersection.approaches.size());
        for (const auto& approach : intersection.approaches) {
            std::vector<flatbuffers::Offset<SynFlatBuffers::Approach::Lane>> flatbuffer_lanes;
            flatbuffer_lanes.reserve(approach->lanes.size());
            for (const auto& lane : approach->lanes) {
                std::vector<flatbuffers::Offset<SynFlatBuffers::Vector>> flatbuffer_control_points;
                flatbuffer_control_points.reserve(lane.controlPoints.size());
                for (const auto& point : lane.controlPoints) {
                    flatbuffer_control_points.push_back(
                        SynFlatBuffers::CreateVector(builder, point.x(), point.y(), point.z()));
                }
                flatbuffer_lanes.push_back(
                    SynFlatBuffers::Approach::CreateLaneDirect(builder, lane.width, &flatbuffer_control_points));
            }
            flatbuffer_approaches.push_back(
                SynFlatBuffers::Approach::CreateStateDirect(builder, approach->time, &flatbuffer_lanes));
        }
        flatbuffer_intersections.push_back(
            SynFlatBuffers::MAP::Createintersection(builder, builder.CreateVector(flatbuffer_approaches)));
    }

    flatbuffers::Offset<SynFlatBuffers::MAP::State> flatbuffer_state =
        SynFlatBuffers::MAP::CreateState(builder, time, builder.CreateVector(flatbuffer_intersections));

    FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(SynFlatBuffers::CreateMessage(
        builder, SynFlatBuffers::Type_MAP_State, flatbuffer_state.Union(), m_source_id, m_destination_id));
    return message;
}

unsigned SynMAPMessage::AddLane(int intersection, int approach, ApproachLane lane) {
    // Adding in blank intersections/approaches until we fill up to the intersection/approach you actually wanted to add
    while (this->intersections.size() <= (size_t)intersection)
        this->intersections.emplace_back();

    while (this->intersections[intersection].approaches.size() <= approach)
        this->intersections[intersection].approaches.push_back(
            chrono_types::make_shared<SynApproachMessage>(m_source_id, m_destination_id));

    unsigned num_lanes = (unsigned)this->intersections[intersection].approaches[approach]->lanes.size();
    this->intersections[intersection].approaches[approach]->lanes.push_back(lane);

    return num_lanes;
}

}  // namespace synchrono
}  // namespace chrono