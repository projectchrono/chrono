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
// Class for an agent that wraps a Chrono::Vehicle tracked vehicle. The
// underlying dynamics are those of a tracked vehicle, state data consists of
// the position and orientation of the chassis and the various track components
/// of the vehicle
//
// =============================================================================

#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

SynTrackedVehicleAgent::SynTrackedVehicleAgent(ChTrackedVehicle* vehicle, const std::string& filename)
    : SynAgent(), m_vehicle(vehicle) {
    m_state = chrono_types::make_shared<SynTrackedVehicleStateMessage>();
    m_description = chrono_types::make_shared<SynTrackedVehicleDescriptionMessage>();

    if (!filename.empty())
        SetZombieVisualizationFilesFromJSON(filename);
}

SynTrackedVehicleAgent::~SynTrackedVehicleAgent() {}

void SynTrackedVehicleAgent::InitializeZombie(ChSystem* system) {
    m_zombie_body = CreateChassisZombieBody(m_description->chassis_vis_file, system);

    auto track_shoe_trimesh = CreateMeshZombieComponent(m_description->track_shoe_vis_file);
    auto left_sprocket_trimesh = CreateMeshZombieComponent(m_description->left_sprocket_vis_file);
    auto right_sprocket_trimesh = CreateMeshZombieComponent(m_description->right_sprocket_vis_file);
    auto left_idler_trimesh = CreateMeshZombieComponent(m_description->left_idler_vis_file);
    auto right_idler_trimesh = CreateMeshZombieComponent(m_description->right_idler_vis_file);
    auto left_road_wheel_trimesh = CreateMeshZombieComponent(m_description->left_road_wheel_vis_file);
    auto right_road_wheel_trimesh = CreateMeshZombieComponent(m_description->right_road_wheel_vis_file);

    m_track_shoe_list.resize(m_description->num_track_shoes);
    m_sprocket_list.resize(m_description->num_sprockets);
    m_idler_list.resize(m_description->num_idlers);
    m_road_wheel_list.resize(m_description->num_road_wheels);

    AddMeshToVector(track_shoe_trimesh, m_track_shoe_list, system);
    AddMeshToVector(left_sprocket_trimesh, right_sprocket_trimesh, m_sprocket_list, system);
    AddMeshToVector(left_idler_trimesh, right_idler_trimesh, m_idler_list, system);
    AddMeshToVector(left_road_wheel_trimesh, right_road_wheel_trimesh, m_road_wheel_list, system);
}

void SynTrackedVehicleAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {
    if (auto state = std::dynamic_pointer_cast<SynTrackedVehicleStateMessage>(message)) {
        m_zombie_body->SetFrame_REF_to_abs(state->chassis.GetFrame());
        for (int i = 0; i < state->track_shoes.size(); i++)
            m_track_shoe_list[i]->SetFrame_REF_to_abs(state->track_shoes[i].GetFrame());
        for (int i = 0; i < state->sprockets.size(); i++)
            m_sprocket_list[i]->SetFrame_REF_to_abs(state->sprockets[i].GetFrame());
        for (int i = 0; i < state->idlers.size(); i++)
            m_idler_list[i]->SetFrame_REF_to_abs(state->idlers[i].GetFrame());
        for (int i = 0; i < state->road_wheels.size(); i++)
            m_road_wheel_list[i]->SetFrame_REF_to_abs(state->road_wheels[i].GetFrame());
    }
}

void SynTrackedVehicleAgent::Update() {
    if (!m_vehicle)
        return;

    SynPose chassis(m_vehicle->GetChassisBody()->GetFrame_REF_to_abs().GetPos(), m_vehicle->GetChassisBody()->GetRot());

    std::vector<SynPose> track_shoes;
    BodyStates left_states(m_vehicle->GetNumTrackShoes(LEFT));
    BodyStates right_states(m_vehicle->GetNumTrackShoes(RIGHT));
    m_vehicle->GetTrackShoeStates(LEFT, left_states);
    m_vehicle->GetTrackShoeStates(RIGHT, right_states);
    for (auto& state : left_states)
        track_shoes.emplace_back(state.pos, state.rot);
    for (auto& state : right_states)
        track_shoes.emplace_back(state.pos, state.rot);

    auto left_assembly = m_vehicle->GetTrackAssembly(LEFT);
    auto right_assembly = m_vehicle->GetTrackAssembly(RIGHT);

    std::vector<SynPose> sprockets;
    sprockets.emplace_back(left_assembly->GetSprocket()->GetGearBody()->GetFrame_REF_to_abs());
    sprockets.emplace_back(right_assembly->GetSprocket()->GetGearBody()->GetFrame_REF_to_abs());

    std::vector<SynPose> idlers;
    idlers.emplace_back(left_assembly->GetIdler()->GetWheelBody()->GetFrame_REF_to_abs());
    idlers.emplace_back(right_assembly->GetIdler()->GetWheelBody()->GetFrame_REF_to_abs());

    std::vector<SynPose> road_wheels;
    for (int i = 0; i < m_vehicle->GetTrackAssembly(LEFT)->GetNumTrackSuspensions(); i++)
        road_wheels.emplace_back(left_assembly->GetRoadWheel(i)->GetBody()->GetFrame_REF_to_abs());

    for (int i = 0; i < m_vehicle->GetTrackAssembly(RIGHT)->GetNumTrackSuspensions(); i++)
        road_wheels.emplace_back(right_assembly->GetRoadWheel(i)->GetBody()->GetFrame_REF_to_abs());

    auto time = m_vehicle->GetSystem()->GetChTime();
    m_state->SetState(time, chassis, track_shoes, sprockets, idlers, road_wheels);
}

// ------------------------------------------------------------------------

void SynTrackedVehicleAgent::SetZombieVisualizationFilesFromJSON(const std::string& filename) {
    m_description->SetZombieVisualizationFilesFromJSON(filename);
}

void SynTrackedVehicleAgent::SetKey(AgentKey agent_key) {
    m_description->SetSourceKey(agent_key);
    m_state->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

// ------------------------------------------------------------------------

std::shared_ptr<ChVisualShapeTriangleMesh> SynTrackedVehicleAgent::CreateMeshZombieComponent(const std::string& filename) {
    auto trimesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    if (!filename.empty()) {
        auto mesh =
            geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(filename), false, false);
        trimesh->SetMesh(mesh);
        trimesh->SetMutable(false);
        trimesh->SetName(filesystem::path(filename).stem());
    }
    return trimesh;
}

std::shared_ptr<ChBodyAuxRef> SynTrackedVehicleAgent::CreateChassisZombieBody(const std::string& filename,
                                                                              ChSystem* system) {
    auto trimesh = CreateMeshZombieComponent(filename);

    auto zombie_body = chrono_types::make_shared<ChBodyAuxRef>();
    zombie_body->AddVisualShape(trimesh);
    zombie_body->SetCollide(false);
    zombie_body->SetBodyFixed(true);
    zombie_body->SetFrame_COG_to_REF(ChFrame<>({0, 0, -0.2}, {1, 0, 0, 0}));
    system->Add(zombie_body);

    return zombie_body;
}

void SynTrackedVehicleAgent::AddMeshToVector(std::shared_ptr<ChVisualShapeTriangleMesh> trimesh,
                                             std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                                             ChSystem* system) {
    for (auto& ref : ref_list) {
        ref = chrono_types::make_shared<ChBodyAuxRef>();
        ref->AddVisualShape(trimesh);
        ref->SetCollide(false);
        ref->SetBodyFixed(true);
        system->Add(ref);
    }
}

void SynTrackedVehicleAgent::AddMeshToVector(std::shared_ptr<ChVisualShapeTriangleMesh> left,
                                             std::shared_ptr<ChVisualShapeTriangleMesh> right,
                                             std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                                             ChSystem* system) {
    for (int i = 0; i < ref_list.size() / 2; i++) {
        auto& ref_left = ref_list[i];
        ref_left = chrono_types::make_shared<ChBodyAuxRef>();
        ref_left->AddVisualShape(left);
        ref_left->SetCollide(false);
        ref_left->SetBodyFixed(true);
        system->Add(ref_left);

        auto& ref_right = ref_list[ref_list.size() - i - 1];
        ref_right = chrono_types::make_shared<ChBodyAuxRef>();
        ref_right->AddVisualShape(right);
        ref_right->SetCollide(false);
        ref_right->SetBodyFixed(true);
        system->Add(ref_right);
    }
}

}  // namespace synchrono
}  // namespace chrono