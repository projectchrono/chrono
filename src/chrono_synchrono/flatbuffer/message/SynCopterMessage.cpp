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
// Authors: Simone Benatti
// =============================================================================
//
// Wraps data from a copter state message into a corresponding C++
// object.
// See also flatbuffer/fbs/Agent.fbs
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynCopterMessage.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;
namespace Copter = SynFlatBuffers::Agent::Copter;

SynCopterStateMessage::SynCopterStateMessage(unsigned int source_id, unsigned int destination_id)
    : SynMessage(source_id, destination_id) {}

void SynCopterStateMessage::SetState(double time, SynPose chassis, std::vector<SynPose> props) {
    this->time = time;
    this->chassis = chassis;
    this->props = props;
}

void SynCopterStateMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Agent::Copter::State
    if (message->message_type() != SynFlatBuffers::Type_Agent_State)
        return;

    m_source_id = message->source_id();
    m_destination_id = message->destination_id();

    auto agent_state = message->message_as_Agent_State();
    auto state = agent_state->message_as_Copter_State();

    time = state->time();
    chassis = SynPose(state->chassis());

    props.clear();
    for (auto prop : (*state->propellers()))
        props.emplace_back(prop);
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynCopterStateMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto flatbuffer_chassis = this->chassis.ToFlatBuffers(builder);

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> flatbuffer_props;
    flatbuffer_props.reserve(this->props.size());
    for (const auto& prop : this->props)
        flatbuffer_props.push_back(prop.ToFlatBuffers(builder));

    auto copter_type = Agent::Type_Copter_State;
    auto copter_state = Copter::CreateStateDirect(builder,             //
                                                  this->time,          //
                                                  flatbuffer_chassis,  //
                                                  &flatbuffer_props)
                            .Union();

    auto flatbuffer_state = Agent::CreateState(builder, copter_type, copter_state);
    auto flatbuffer_message = SynFlatBuffers::CreateMessage(builder,                           //
                                                            SynFlatBuffers::Type_Agent_State,  //
                                                            flatbuffer_state.Union(),          //
                                                            m_source_id, m_destination_id);    //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

// ---------------------------------------------------------------------------------------------------------------

SynCopterDescriptionMessage::SynCopterDescriptionMessage(unsigned int source_id, unsigned int destination_id)
    : SynMessage(source_id, destination_id) {}

/// Generate agent description from FlatBuffers message
void SynCopterDescriptionMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    /// Cast from SynFlatBuffers::Message to SynFlatBuffers::Agent::Copter::Description
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;

    auto description = message->message_as_Agent_Description();
    m_source_id = message->source_id();
    m_destination_id = message->destination_id();
    auto copter_description = description->description_as_Copter_Description();
    SetVisualizationFiles(copter_description->chassis_vis_file()->str(),
                          copter_description->propeller_vis_file()->str());

    SetNumProps(copter_description->num_props());
}

/// Generate FlatBuffers message from this agent's description
FlatBufferMessage SynCopterDescriptionMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto flatbuffer_type = Agent::Type_Copter_Description;

    flatbuffers::Offset<Copter::Description> copter_description = 0;
    copter_description = Copter::CreateDescriptionDirect(builder,                           //
                                                         this->chassis_vis_file.c_str(),    //
                                                         this->propeller_vis_file.c_str(),  //
                                                         this->num_props);                  //

    auto flatbuffer_description = Agent::CreateDescription(builder,                    //
                                                           flatbuffer_type,            //
                                                           copter_description.Union()  //
    );                                                                                 //

    return SynFlatBuffers::CreateMessage(builder,                                 //
                                         SynFlatBuffers::Type_Agent_Description,  //
                                         flatbuffer_description.Union(),          //
                                         m_source_id, m_destination_id);          //
}

void SynCopterDescriptionMessage::SetVisualizationFiles(const std::string& chassis_vis_file,
                                                        const std::string& propeller_vis_file) {
    this->chassis_vis_file = chassis_vis_file;
    this->propeller_vis_file = propeller_vis_file;
}

void SynCopterDescriptionMessage::SetNumProps(int num_props) {
    this->num_props = num_props;
}

}  // namespace synchrono
}  // namespace chrono
