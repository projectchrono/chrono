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
// Wraps data from a wheeled vehicle state message into a corresponding C++
// object.
// See also flatbuffer/fbs/Agent.fbs
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;
namespace WheeledVehicle = SynFlatBuffers::Agent::WheeledVehicle;

/// Constructors
SynWheeledVehicleMessage::SynWheeledVehicleMessage(int rank,
                                                   std::string json,
                                                   std::shared_ptr<SynWheeledVehicleState> state,
                                                   std::shared_ptr<SynWheeledVehicleDescription> description)
    : SynAgentMessage(rank, SynMessageType::WHEELED_VEHICLE, json) {
    m_state = state ? state : chrono_types::make_shared<SynWheeledVehicleState>();
    m_description = m_vehicle_description = description ? description  //
                                                        : chrono_types::make_shared<SynWheeledVehicleDescription>();
}

SynWheeledVehicleMessage::SynWheeledVehicleMessage(int rank,
                                                   std::shared_ptr<SynWheeledVehicleState> state,
                                                   std::shared_ptr<SynWheeledVehicleDescription> description)
    : SynAgentMessage(rank, SynMessageType::WHEELED_VEHICLE) {
    m_state = state;
    m_description = m_vehicle_description = description ? description  //
                                                        : chrono_types::make_shared<SynWheeledVehicleDescription>();
}

// ---------------------------------------------------------------------------------------------------------------

void SynWheeledVehicleMessage::StateFromMessage(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Agent::WheeledVehicle::State
    if (message->message_type() != SynFlatBuffers::Type_Agent_State)
        return;

    auto agent_state = message->message_as_Agent_State();
    auto state = agent_state->message_as_WheeledVehicle_State();

    std::vector<SynPose> wheels;
    for (auto wheel : (*state->wheels()))
        wheels.emplace_back(wheel);

    SynPose chassis(state->chassis());

    m_state->SetState(state->time(), chassis, wheels);
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynWheeledVehicleMessage::MessageFromState(flatbuffers::FlatBufferBuilder& builder) {
    auto chassis = m_state->chassis.ToFlatBuffers(builder);

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> wheels;
    for (auto& wheel : m_state->wheels)
        wheels.push_back(wheel.ToFlatBuffers(builder));

    auto vehicle_type = Agent::Type_WheeledVehicle_State;
    auto vehicle_state = WheeledVehicle::CreateStateDirect(builder, m_state->time, chassis, &wheels).Union();

    auto flatbuffer_state = Agent::CreateState(builder, vehicle_type, vehicle_state);
    auto flatbuffer_message = SynFlatBuffers::CreateMessage(builder,                           //
                                                            SynFlatBuffers::Type_Agent_State,  //
                                                            flatbuffer_state.Union(),          //
                                                            m_rank);                           //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

// ---------------------------------------------------------------------------------------------------------------

/// Generate agent description from FlatBuffers message
void SynWheeledVehicleMessage::DescriptionFromMessage(const SynFlatBuffers::Message* message) {
    /// Cast from SynFlatBuffers::Message to SynFlatBuffers::Agent::WheeledVehicle::Description
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;

    auto description = message->message_as_Agent_Description();
    m_rank = message->rank();

    if (description->json()->Length())
        m_vehicle_description->json = description->json()->str();
    else {
        auto vehicle_description = description->description_as_WheeledVehicle_Description();

        m_vehicle_description->SetVisualizationFiles(vehicle_description->chassis_vis_file()->str(),
                                                     vehicle_description->wheel_vis_file()->str(),
                                                     vehicle_description->tire_vis_file()->str());
        m_vehicle_description->SetNumWheels(vehicle_description->num_wheels());
    }
}

/// Generate FlatBuffers message from this agent's description
FlatBufferMessage SynWheeledVehicleMessage::MessageFromDescription(flatbuffers::FlatBufferBuilder& builder) {
    auto flatbuffer_json = builder.CreateString(m_vehicle_description->json);
    auto flatbuffer_type = Agent::Type_WheeledVehicle_Description;

    flatbuffers::Offset<WheeledVehicle::Description> vehicle_description = 0;
    if (m_vehicle_description->json.empty()) {
        vehicle_description =
            WheeledVehicle::CreateDescriptionDirect(builder,                                            //
                                                    m_vehicle_description->m_chassis_vis_file.c_str(),  //
                                                    m_vehicle_description->m_wheel_vis_file.c_str(),    //
                                                    m_vehicle_description->m_tire_vis_file.c_str(),     //
                                                    m_vehicle_description->m_num_wheels);               //
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
