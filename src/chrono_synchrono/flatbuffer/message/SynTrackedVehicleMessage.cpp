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
// Authors: Aaron Young
// =============================================================================
//
// Wraps data received from a tracked vehicle flatbuffer state message, into a
// corresponding C++ object.
// See also flatbuffer/fbs/Agent.fbs
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynTrackedVehicleMessage.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;
namespace TrackedVehicle = SynFlatBuffers::Agent::TrackedVehicle;

/// Constructors
SynTrackedVehicleMessage::SynTrackedVehicleMessage(int rank,
                                                   std::string json,
                                                   std::shared_ptr<SynTrackedVehicleState> state,
                                                   std::shared_ptr<SynTrackedVehicleDescription> description)
    : SynAgentMessage(rank, SynMessageType::TRACKED_VEHICLE, json) {
    m_state = state ? state : chrono_types::make_shared<SynTrackedVehicleState>();
    m_description = m_vehicle_description = description ? description  //
                                                        : chrono_types::make_shared<SynTrackedVehicleDescription>();
}

SynTrackedVehicleMessage::SynTrackedVehicleMessage(int rank,
                                                   std::shared_ptr<SynTrackedVehicleState> state,
                                                   std::shared_ptr<SynTrackedVehicleDescription> description)
    : SynAgentMessage(rank, SynMessageType::TRACKED_VEHICLE) {
    m_state = state;
    m_description = m_vehicle_description = description ? description  //
                                                        : chrono_types::make_shared<SynTrackedVehicleDescription>();
}

void SynTrackedVehicleMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Agent::TrackedVehicle::State
    if (message->message_type() != SynFlatBuffers::Type_Agent_State)
        return;

    auto agent_state = message->message_as_Agent_State();
    auto state = agent_state->message_as_TrackedVehicle_State();

    SynPose chassis(state->chassis());

    std::vector<SynPose> track_shoes;
    for (auto track_shoe : (*state->track_shoes()))
        track_shoes.emplace_back(track_shoe);

    std::vector<SynPose> sprockets;
    for (auto sprocket : (*state->sprockets()))
        sprockets.emplace_back(sprocket);

    std::vector<SynPose> idlers;
    for (auto idler : (*state->idlers()))
        idlers.emplace_back(idler);

    std::vector<SynPose> road_wheels;
    for (auto road_wheel : (*state->road_wheels()))
        road_wheels.emplace_back(road_wheel);

    m_state->SetState(state->time(), chassis, track_shoes, sprockets, idlers, road_wheels);
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynTrackedVehicleMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    auto chassis = m_state->chassis.ToFlatBuffers(builder);

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> track_shoes;
    for (auto& track_shoe : m_state->track_shoes)
        track_shoes.push_back(track_shoe.ToFlatBuffers(builder));

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> sprockets;
    for (auto& sprocket : m_state->sprockets)
        sprockets.push_back(sprocket.ToFlatBuffers(builder));

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> idlers;
    for (auto& idler : m_state->idlers)
        idlers.push_back(idler.ToFlatBuffers(builder));

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> road_wheels;
    for (auto& road_wheel : m_state->road_wheels)
        road_wheels.push_back(road_wheel.ToFlatBuffers(builder));

    auto vehicle_type = Agent::Type_TrackedVehicle_State;
    auto vehicle_state = TrackedVehicle::CreateStateDirect(builder,        //
                                                           m_state->time,  //
                                                           chassis,        //
                                                           &track_shoes,   //
                                                           &sprockets,     //
                                                           &idlers,        //
                                                           &road_wheels);  //

    auto flatbuffer_state = Agent::CreateState(builder, vehicle_type, vehicle_state.Union());
    auto flatbuffer_message = SynFlatBuffers::CreateMessage(builder,                           //
                                                            SynFlatBuffers::Type_Agent_State,  //
                                                            flatbuffer_state.Union(),          //
                                                            m_rank);                           //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

/// Generate description from FlatBuffers message
void SynTrackedVehicleMessage::DescriptionFromMessage(const SynFlatBuffers::Message* message) {
    /// Cast from SynFlatBuffers::Message to SynFlatBuffers::Agent::TrackedVehicle::Description
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;

    auto description = message->message_as_Agent_Description();
    m_rank = message->rank();

    if (description->json()->Length())
        m_vehicle_description->json = description->json()->str();
    else {
        auto vehicle_description = description->description_as_TrackedVehicle_Description();

        m_vehicle_description->SetVisualizationFiles(vehicle_description->chassis_vis_file()->str(),            //
                                                     vehicle_description->track_shoe_vis_file()->str(),         //
                                                     vehicle_description->left_sprocket_vis_file()->str(),      //
                                                     vehicle_description->right_sprocket_vis_file()->str(),     //
                                                     vehicle_description->left_idler_vis_file()->str(),         //
                                                     vehicle_description->right_idler_vis_file()->str(),        //
                                                     vehicle_description->left_road_wheel_vis_file()->str(),    //
                                                     vehicle_description->right_road_wheel_vis_file()->str());  //

        m_vehicle_description->SetNumAssemblyComponents(vehicle_description->num_track_shoes(),   //
                                                        vehicle_description->num_sprockets(),     //
                                                        vehicle_description->num_idlers(),        //
                                                        vehicle_description->num_road_wheels());  //
    }
}

/// Generate FlatBuffers message from this agent's description
FlatBufferMessage SynTrackedVehicleMessage::MessageFromDescription(flatbuffers::FlatBufferBuilder& builder) {
    auto flatbuffer_json = builder.CreateString(m_vehicle_description->json);
    auto flatbuffer_type = Agent::Type_TrackedVehicle_Description;

    flatbuffers::Offset<TrackedVehicle::Description> vehicle_description = 0;
    if (m_vehicle_description->json.empty()) {
        vehicle_description =
            TrackedVehicle::CreateDescriptionDirect(builder,
                                                    m_vehicle_description->m_chassis_vis_file.c_str(),           //
                                                    m_vehicle_description->m_track_shoe_vis_file.c_str(),        //
                                                    m_vehicle_description->m_left_sprocket_vis_file.c_str(),     //
                                                    m_vehicle_description->m_right_sprocket_vis_file.c_str(),    //
                                                    m_vehicle_description->m_left_idler_vis_file.c_str(),        //
                                                    m_vehicle_description->m_right_idler_vis_file.c_str(),       //
                                                    m_vehicle_description->m_left_road_wheel_vis_file.c_str(),   //
                                                    m_vehicle_description->m_right_road_wheel_vis_file.c_str(),  //
                                                    m_vehicle_description->m_num_track_shoes,                    //
                                                    m_vehicle_description->m_num_sprockets,                      //
                                                    m_vehicle_description->m_num_idlers,                         //
                                                    m_vehicle_description->m_num_road_wheels);                   //
    }

    auto flatbuffer_description = Agent::CreateDescription(builder,                      //
                                                           flatbuffer_type,              //
                                                           vehicle_description.Union(),  //
                                                           flatbuffer_json);             //

    return SynFlatBuffers::CreateMessage(builder,                                 //
                                         SynFlatBuffers::Type_Agent_Description,  //
                                         flatbuffer_description.Union(),          //
                                         m_rank);                                 //
}

}  // namespace synchrono
}  // namespace chrono
