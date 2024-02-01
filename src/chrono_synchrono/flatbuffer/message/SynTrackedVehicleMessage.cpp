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

#include "chrono_vehicle/utils/ChUtilsJSON.h"

namespace chrono {
namespace synchrono {

namespace Agent = SynFlatBuffers::Agent;
namespace TrackedVehicle = SynFlatBuffers::Agent::TrackedVehicle;

SynTrackedVehicleStateMessage::SynTrackedVehicleStateMessage(AgentKey source_key, AgentKey destination_key)
    : SynMessage(source_key, destination_key) {}

void SynTrackedVehicleStateMessage::SetState(double time,
                                             SynPose chassis,
                                             std::vector<SynPose> track_shoes,
                                             std::vector<SynPose> sprockets,
                                             std::vector<SynPose> idlers,
                                             std::vector<SynPose> road_wheels) {
    this->time = time;
    this->chassis = chassis;
    this->track_shoes = track_shoes;
    this->sprockets = sprockets;
    this->idlers = idlers;
    this->road_wheels = road_wheels;
}

void SynTrackedVehicleStateMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    // System of casts from SynFlatBuffers::Message to SynFlatBuffers::Agent::TrackedVehicle::State
    if (message->message_type() != SynFlatBuffers::Type_Agent_State)
        return;

    m_source_key = AgentKey(message->source_key());
    m_destination_key = AgentKey(message->destination_key());

    auto agent_state = message->message_as_Agent_State();
    auto state = agent_state->message_as_TrackedVehicle_State();

    this->chassis = SynPose(state->chassis());

    this->track_shoes.clear();
    for (auto track_shoe : (*state->track_shoes()))
        this->track_shoes.emplace_back(track_shoe);

    this->sprockets.clear();
    for (auto sprocket : (*state->sprockets()))
        this->sprockets.emplace_back(sprocket);

    this->idlers.clear();
    for (auto idler : (*state->idlers()))
        this->idlers.emplace_back(idler);

    this->road_wheels.clear();
    for (auto road_wheel : (*state->road_wheels()))
        this->road_wheels.emplace_back(road_wheel);
}

/// Generate FlatBuffers message from this message's state
FlatBufferMessage SynTrackedVehicleStateMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    auto chassis = this->chassis.ToFlatBuffers(builder);

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> track_shoes;
    track_shoes.reserve(this->track_shoes.size());
    for (const auto& track_shoe : this->track_shoes)
        track_shoes.push_back(track_shoe.ToFlatBuffers(builder));

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> sprockets;
    sprockets.reserve(this->sprockets.size());
    for (const auto& sprocket : this->sprockets)
        sprockets.push_back(sprocket.ToFlatBuffers(builder));

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> idlers;
    idlers.reserve(this->idlers.size());
    for (const auto& idler : this->idlers)
        idlers.push_back(idler.ToFlatBuffers(builder));

    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> road_wheels;
    road_wheels.reserve(this->road_wheels.size());
    for (const auto& road_wheel : this->road_wheels)
        road_wheels.push_back(road_wheel.ToFlatBuffers(builder));

    auto vehicle_type = Agent::Type_TrackedVehicle_State;
    auto vehicle_state = TrackedVehicle::CreateStateDirect(builder,        //
                                                           this->time,     //
                                                           chassis,        //
                                                           &track_shoes,   //
                                                           &sprockets,     //
                                                           &idlers,        //
                                                           &road_wheels);  //

    auto flatbuffer_state = Agent::CreateState(builder, vehicle_type, vehicle_state.Union());
    auto flatbuffer_message =
        SynFlatBuffers::CreateMessage(builder,                                                                   //
                                      SynFlatBuffers::Type_Agent_State,                                          //
                                      flatbuffer_state.Union(),                                                  //
                                      m_source_key.GetFlatbuffersKey(), m_destination_key.GetFlatbuffersKey());  //

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

// ---------------------------------------------------------------------------------------------------------------

SynTrackedVehicleDescriptionMessage::SynTrackedVehicleDescriptionMessage(AgentKey source_key,
                                                                         AgentKey destination_key,
                                                                         const std::string& json)
    : SynMessage(source_key, destination_key), json(json) {}

/// Generate description from FlatBuffers message
void SynTrackedVehicleDescriptionMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    /// Cast from SynFlatBuffers::Message to SynFlatBuffers::Agent::TrackedVehicle::Description
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;

    auto description = message->message_as_Agent_Description();
    m_source_key = AgentKey(message->source_key());
    m_destination_key = message->destination_key();

    if (description->json()->size())
        this->json = description->json()->str();
    else {
        auto vehicle_description = description->description_as_TrackedVehicle_Description();

        SetVisualizationFiles(vehicle_description->chassis_vis_file()->str(),            //
                              vehicle_description->track_shoe_vis_file()->str(),         //
                              vehicle_description->left_sprocket_vis_file()->str(),      //
                              vehicle_description->right_sprocket_vis_file()->str(),     //
                              vehicle_description->left_idler_vis_file()->str(),         //
                              vehicle_description->right_idler_vis_file()->str(),        //
                              vehicle_description->left_road_wheel_vis_file()->str(),    //
                              vehicle_description->right_road_wheel_vis_file()->str());  //

        SetNumAssemblyComponents(vehicle_description->num_track_shoes(),   //
                                 vehicle_description->num_sprockets(),     //
                                 vehicle_description->num_idlers(),        //
                                 vehicle_description->num_road_wheels());  //
    }
}

/// Generate FlatBuffers message from this agent's description
FlatBufferMessage SynTrackedVehicleDescriptionMessage::ConvertToFlatBuffers(
    flatbuffers::FlatBufferBuilder& builder) const {
    auto flatbuffer_json = builder.CreateString(this->json);
    auto flatbuffer_type = Agent::Type_TrackedVehicle_Description;

    flatbuffers::Offset<TrackedVehicle::Description> vehicle_description = 0;
    if (this->json.empty()) {
        vehicle_description = TrackedVehicle::CreateDescriptionDirect(builder,
                                                                      this->chassis_vis_file.c_str(),           //
                                                                      this->track_shoe_vis_file.c_str(),        //
                                                                      this->left_sprocket_vis_file.c_str(),     //
                                                                      this->right_sprocket_vis_file.c_str(),    //
                                                                      this->left_idler_vis_file.c_str(),        //
                                                                      this->right_idler_vis_file.c_str(),       //
                                                                      this->left_road_wheel_vis_file.c_str(),   //
                                                                      this->right_road_wheel_vis_file.c_str(),  //
                                                                      this->num_track_shoes,                    //
                                                                      this->num_sprockets,                      //
                                                                      this->num_idlers,                         //
                                                                      this->num_road_wheels);                   //
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

void SynTrackedVehicleDescriptionMessage::SetZombieVisualizationFilesFromJSON(const std::string& filename) {
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
    assert(subtype.compare("TrackedVehicle") == 0);

    // ----------------------------
    // Validations of the JSON file
    // ----------------------------
    assert(d.HasMember("Zombie"));

    assert(d["Zombie"].HasMember("Chassis Visualization File"));
    assert(d["Zombie"].HasMember("Track Shoe Visualization File"));
    assert(d["Zombie"].HasMember("Left Sprocket Visualization File"));
    assert(d["Zombie"].HasMember("Right Sprocket Visualization File"));
    assert(d["Zombie"].HasMember("Left Idler Visualization File"));
    assert(d["Zombie"].HasMember("Right Idler Visualization File"));
    assert(d["Zombie"].HasMember("Left Road Wheel Visualization File"));
    assert(d["Zombie"].HasMember("Right Road Wheel Visualization File"));

    assert(d["Zombie"].HasMember("Number of Track Shoes"));
    assert(d["Zombie"].HasMember("Number of Sprockets"));
    assert(d["Zombie"].HasMember("Number of Idlers"));
    assert(d["Zombie"].HasMember("Number of Road Wheels"));

    // Set the zombie visualization files
    this->chassis_vis_file = d["Zombie"]["Chassis Visualization File"].GetString();
    this->track_shoe_vis_file = d["Zombie"]["Track Shoe Visualization File"].GetString();
    this->left_sprocket_vis_file = d["Zombie"]["Left Sprocket Visualization File"].GetString();
    this->right_sprocket_vis_file = d["Zombie"]["Right Sprocket Visualization File"].GetString();
    this->left_idler_vis_file = d["Zombie"]["Left Idler Visualization File"].GetString();
    this->right_idler_vis_file = d["Zombie"]["Right Idler Visualization File"].GetString();
    this->left_road_wheel_vis_file = d["Zombie"]["Left Road Wheel Visualization File"].GetString();
    this->right_road_wheel_vis_file = d["Zombie"]["Right Road Wheel Visualization File"].GetString();

    // Set number of assembly components
    this->num_track_shoes = d["Zombie"]["Number of Track Shoes"].GetInt();
    this->num_sprockets = d["Zombie"]["Number of Sprockets"].GetInt();
    this->num_idlers = d["Zombie"]["Number of Idlers"].GetInt();
    this->num_road_wheels = d["Zombie"]["Number of Road Wheels"].GetInt();
}

void SynTrackedVehicleDescriptionMessage::SetVisualizationFiles(const std::string& chassis_vis_file,
                                                                const std::string& track_shoe_vis_file,
                                                                const std::string& left_sprocket_vis_file,
                                                                const std::string& right_sprocket_vis_file,
                                                                const std::string& left_idler_vis_file,
                                                                const std::string& right_idler_vis_file,
                                                                const std::string& left_road_wheel_vis_file,
                                                                const std::string& right_road_wheel_vis_file) {
    this->chassis_vis_file = chassis_vis_file;
    this->track_shoe_vis_file = track_shoe_vis_file;
    this->left_sprocket_vis_file = left_sprocket_vis_file;
    this->right_sprocket_vis_file = right_sprocket_vis_file;
    this->left_idler_vis_file = left_idler_vis_file;
    this->right_idler_vis_file = right_idler_vis_file;
    this->left_road_wheel_vis_file = left_road_wheel_vis_file;
    this->right_road_wheel_vis_file = right_road_wheel_vis_file;
}

void SynTrackedVehicleDescriptionMessage::SetNumAssemblyComponents(int num_track_shoes,
                                                                   int num_sprockets,
                                                                   int num_idlers,
                                                                   int num_road_wheels) {
    this->num_track_shoes = num_track_shoes;
    this->num_sprockets = num_sprockets;
    this->num_idlers = num_idlers;
    this->num_road_wheels = num_road_wheels;
}

}  // namespace synchrono
}  // namespace chrono
