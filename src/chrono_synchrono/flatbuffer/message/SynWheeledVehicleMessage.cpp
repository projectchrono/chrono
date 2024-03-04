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

#include "chrono_vehicle/utils/ChUtilsJSON.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;
namespace WheeledVehicle = SynFlatBuffers::Agent::WheeledVehicle;

SynWheeledVehicleStateMessage::SynWheeledVehicleStateMessage(AgentKey source_key, AgentKey destination_key)
    : SynMessage(source_key, destination_key) {}

void SynWheeledVehicleStateMessage::SetState(double time, SynPose chassis, std::vector<SynPose> wheels) {
    this->time = time;
    this->chassis = chassis;
    this->wheels = wheels;
}

void SynWheeledVehicleStateMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Agent::WheeledVehicle::State
    if (message->message_type() != SynFlatBuffers::Type_Agent_State)
        return;

    m_source_key = AgentKey(message->source_key());
    m_destination_key = message->destination_key();

    auto agent_state = message->message_as_Agent_State();
    auto state = agent_state->message_as_WheeledVehicle_State();

    time = state->time();
    chassis = SynPose(state->chassis());

    wheels.clear();
    for (auto wheel : (*state->wheels()))
        wheels.emplace_back(wheel);
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynWheeledVehicleStateMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto flatbuffer_chassis = this->chassis.ToFlatBuffers(builder);

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> flatbuffer_wheels;
    flatbuffer_wheels.reserve(this->wheels.size());
    for (const auto& wheel : this->wheels)
        flatbuffer_wheels.push_back(wheel.ToFlatBuffers(builder));

    auto vehicle_type = Agent::Type_WheeledVehicle_State;
    auto vehicle_state =
        WheeledVehicle::CreateStateDirect(builder, this->time, flatbuffer_chassis, &flatbuffer_wheels).Union();

    auto flatbuffer_state = Agent::CreateState(builder, vehicle_type, vehicle_state);
    auto flatbuffer_message =
        SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Agent_State, flatbuffer_state.Union(),
                                      m_source_key.GetFlatbuffersKey(), m_destination_key.GetFlatbuffersKey());

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

// ---------------------------------------------------------------------------------------------------------------

SynWheeledVehicleDescriptionMessage::SynWheeledVehicleDescriptionMessage(AgentKey source_key,
                                                                         AgentKey destination_key,
                                                                         const std::string& json)
    : SynMessage(source_key, destination_key), json(json) {}

/// Generate agent description from FlatBuffers message
void SynWheeledVehicleDescriptionMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    /// Cast from SynFlatBuffers::Message to SynFlatBuffers::Agent::WheeledVehicle::Description
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;

    auto description = message->message_as_Agent_Description();
    m_source_key = AgentKey(message->source_key());
    m_destination_key = message->destination_key();

    if (description->json()->size())
        this->json = description->json()->str();
    else {
        auto vehicle_description = description->description_as_WheeledVehicle_Description();

        SetVisualizationFiles(vehicle_description->chassis_vis_file()->str(),
                              vehicle_description->wheel_vis_file()->str(),
                              vehicle_description->tire_vis_file()->str());
        SetNumWheels(vehicle_description->num_wheels());
    }
}

/// Generate FlatBuffers message from this agent's description
FlatBufferMessage SynWheeledVehicleDescriptionMessage::ConvertToFlatBuffers(
    flatbuffers::FlatBufferBuilder& builder) const {
    auto flatbuffer_json = builder.CreateString(this->json);
    auto flatbuffer_type = Agent::Type_WheeledVehicle_Description;

    flatbuffers::Offset<WheeledVehicle::Description> vehicle_description = 0;
    if (this->json.empty()) {
        vehicle_description = WheeledVehicle::CreateDescriptionDirect(builder,                         //
                                                                      this->chassis_vis_file.c_str(),  //
                                                                      this->wheel_vis_file.c_str(),    //
                                                                      this->tire_vis_file.c_str(),     //
                                                                      this->num_wheels);               //
    }

    auto flatbuffer_description = Agent::CreateDescription(builder,                      //
                                                           flatbuffer_type,              //
                                                           vehicle_description.Union(),  //
                                                           flatbuffer_json);             //

    return SynFlatBuffers::CreateMessage(builder,                                                                   //
                                         SynFlatBuffers::Type_Agent_Description,                                    //
                                         flatbuffer_description.Union(),                                            //
                                         m_source_key.GetFlatbuffersKey(), m_destination_key.GetFlatbuffersKey());  //
}

void SynWheeledVehicleDescriptionMessage::SetZombieVisualizationFilesFromJSON(const std::string& filename) {
    // Open and parse the input file
    rapidjson::Document d;
    vehicle::ReadFileJSON(filename, d);
    if (d.IsNull())
        throw ChException("Vehicle file not read properly in SetZombieVisualizationFilesFromJSON.");

    // Read top-level data
    assert(d.HasMember("Name"));
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    std::string subtype = d["Template"].GetString();
    assert(type.compare("Vehicle") == 0);
    assert(subtype.compare("WheeledVehicle") == 0);

    // ----------------------------
    // Validations of the JSON file
    // ----------------------------
    assert(d.HasMember("Zombie"));

    assert(d["Zombie"].HasMember("Chassis Visualization File"));
    assert(d["Zombie"].HasMember("Wheel Visualization File"));
    assert(d["Zombie"].HasMember("Tire Visualization File"));

    assert(d["Zombie"].HasMember("Number of Wheels"));

    // Set the zombie visualization files
    this->chassis_vis_file = d["Zombie"]["Chassis Visualization File"].GetString();
    this->wheel_vis_file = d["Zombie"]["Wheel Visualization File"].GetString();
    this->tire_vis_file = d["Zombie"]["Tire Visualization File"].GetString();

    // Set number of wheels
    this->num_wheels = d["Zombie"]["Number of Wheels"].GetInt();
}

void SynWheeledVehicleDescriptionMessage::SetVisualizationFiles(const std::string& chassis_vis_file,
                                                                const std::string& wheel_vis_file,
                                                                const std::string& tire_vis_file) {
    this->chassis_vis_file = chassis_vis_file;
    this->wheel_vis_file = wheel_vis_file;
    this->tire_vis_file = tire_vis_file;
}

void SynWheeledVehicleDescriptionMessage::SetNumWheels(int num_wheels) {
    this->num_wheels = num_wheels;
}

}  // namespace synchrono
}  // namespace chrono
