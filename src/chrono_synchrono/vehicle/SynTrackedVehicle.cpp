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
// Wrapper class for ChTrackedVehicles. Additional functions here are related to
// initializing this as a zombie (setting visual representations, treads)
//
// =============================================================================

#include "chrono_synchrono/vehicle/SynTrackedVehicle.h"

#include "chrono/core/ChLog.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/solver/ChSolverBB.h"

using namespace chrono::vehicle;
using namespace rapidjson;

namespace chrono {
namespace synchrono {

template class SynCustomTrackedVehicle<class chrono::vehicle::m113::M113>;

SynTrackedVehicle::SynTrackedVehicle() : SynVehicle() {
    m_state = chrono_types::make_shared<SynTrackedVehicleState>();
    m_description = chrono_types::make_shared<SynTrackedVehicleDescription>();
}

SynTrackedVehicle::SynTrackedVehicle(ChTrackedVehicle* tracked_vehicle)
    : SynVehicle(false), m_tracked_vehicle(tracked_vehicle) {
    if (!tracked_vehicle)
        throw ChException("SynTrackedVehicle constructor - tracked_vehicle is NULL");
    m_system = m_tracked_vehicle->GetSystem();

    m_state = chrono_types::make_shared<SynTrackedVehicleState>();
    m_description = chrono_types::make_shared<SynTrackedVehicleDescription>();

    m_shoe_forces_left = TerrainForces(m_tracked_vehicle->GetNumTrackShoes(LEFT));
    m_shoe_forces_right = TerrainForces(m_tracked_vehicle->GetNumTrackShoes(RIGHT));
}

SynTrackedVehicle::SynTrackedVehicle(const ChCoordsys<>& coord_sys,
                                     const std::string& filename,
                                     ChContactMethod contact_method)
    : SynVehicle(filename, contact_method) {
    m_state = chrono_types::make_shared<SynTrackedVehicleState>();
    m_description = chrono_types::make_shared<SynTrackedVehicleDescription>();

    CreateVehicle(coord_sys, filename, m_system);
}

SynTrackedVehicle::SynTrackedVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system)
    : SynVehicle(filename, system) {
    m_state = chrono_types::make_shared<SynTrackedVehicleState>();
    m_description = chrono_types::make_shared<SynTrackedVehicleDescription>();

    CreateVehicle(coord_sys, filename, system);
}

SynTrackedVehicle::SynTrackedVehicle(const std::string& filename) : SynVehicle(filename) {
    m_state = chrono_types::make_shared<SynTrackedVehicleState>();
    m_description = chrono_types::make_shared<SynTrackedVehicleDescription>();

    CreateZombie(filename);
}

SynTrackedVehicle::~SynTrackedVehicle() {
    if (m_tracked_vehicle && m_owns_vehicle) {
        delete m_tracked_vehicle;

        if (m_system)
            delete m_system;
    }
}

void SynTrackedVehicle::InitializeZombie(ChSystem* system) {
    CreateChassisZombieBody(m_description->m_chassis_vis_file, system);

    auto track_shoe_trimesh = CreateMeshZombieComponent(m_description->m_track_shoe_vis_file);
    auto left_sprocket_trimesh = CreateMeshZombieComponent(m_description->m_left_sprocket_vis_file);
    auto right_sprocket_trimesh = CreateMeshZombieComponent(m_description->m_right_sprocket_vis_file);
    auto left_idler_trimesh = CreateMeshZombieComponent(m_description->m_left_idler_vis_file);
    auto right_idler_trimesh = CreateMeshZombieComponent(m_description->m_right_idler_vis_file);
    auto left_road_wheel_trimesh = CreateMeshZombieComponent(m_description->m_left_road_wheel_vis_file);
    auto right_road_wheel_trimesh = CreateMeshZombieComponent(m_description->m_right_road_wheel_vis_file);

    m_track_shoe_list.resize(m_description->m_num_track_shoes);
    m_sprocket_list.resize(m_description->m_num_sprockets);
    m_idler_list.resize(m_description->m_num_idlers);
    m_road_wheel_list.resize(m_description->m_num_road_wheels);

    AddMeshToVector(track_shoe_trimesh, m_track_shoe_list, system);
    AddMeshToVector(left_sprocket_trimesh, right_sprocket_trimesh, m_sprocket_list, system);
    AddMeshToVector(left_idler_trimesh, right_idler_trimesh, m_idler_list, system);
    AddMeshToVector(left_road_wheel_trimesh, right_road_wheel_trimesh, m_road_wheel_list, system);

    m_system = system;
}

void SynTrackedVehicle::SynchronizeZombie(SynMessage* message) {
    if (m_zombie_body == nullptr)
        return;

    if (message != nullptr) {
        if (message->GetType() != SynMessageType::TRACKED_VEHICLE)
            return;

        m_state = ((SynTrackedVehicleMessage*)message)->GetTrackedState();
    } else {
        // Dead reckon if state is not received
        // m_state->chassis.Step(HEARTBEAT);
        // for (auto& track_shoe : m_state->track_shoes)
        //     track_shoe.Step(HEARTBEAT);
        // for (auto& sprocket : m_state->sprockets)
        //     sprocket.Step(HEARTBEAT);
        // for (auto& idler : m_state->idlers)
        //     idler.Step(HEARTBEAT);
        // for (auto& road_wheel : m_state->road_wheels)
        //     road_wheel.Step(HEARTBEAT);
    }

    m_zombie_body->SetFrame_REF_to_abs(m_state->chassis.GetFrame());
    for (int i = 0; i < m_state->track_shoes.size(); i++)
        m_track_shoe_list[i]->SetFrame_REF_to_abs(m_state->track_shoes[i].GetFrame());
    for (int i = 0; i < m_state->sprockets.size(); i++)
        m_sprocket_list[i]->SetFrame_REF_to_abs(m_state->sprockets[i].GetFrame());
    for (int i = 0; i < m_state->idlers.size(); i++)
        m_idler_list[i]->SetFrame_REF_to_abs(m_state->idlers[i].GetFrame());
    for (int i = 0; i < m_state->road_wheels.size(); i++)
        m_road_wheel_list[i]->SetFrame_REF_to_abs(m_state->road_wheels[i].GetFrame());
}

void SynTrackedVehicle::Update() {
    SynPose chassis(m_tracked_vehicle->GetChassisBody()->GetFrame_REF_to_abs().GetPos(),
                    m_tracked_vehicle->GetChassisBody()->GetRot());

    std::vector<SynPose> track_shoes;
    BodyStates left_states(m_tracked_vehicle->GetNumTrackShoes(LEFT));
    BodyStates right_states(m_tracked_vehicle->GetNumTrackShoes(RIGHT));
    m_tracked_vehicle->GetTrackShoeStates(LEFT, left_states);
    m_tracked_vehicle->GetTrackShoeStates(RIGHT, right_states);
    for (auto& state : left_states)
        track_shoes.emplace_back(state.pos, state.rot);
    for (auto& state : right_states)
        track_shoes.emplace_back(state.pos, state.rot);

    auto left_assembly = m_tracked_vehicle->GetTrackAssembly(LEFT);
    auto right_assembly = m_tracked_vehicle->GetTrackAssembly(RIGHT);

    std::vector<SynPose> sprockets;
    sprockets.emplace_back(left_assembly->GetSprocket()->GetGearBody()->GetFrame_REF_to_abs());
    sprockets.emplace_back(right_assembly->GetSprocket()->GetGearBody()->GetFrame_REF_to_abs());

    std::vector<SynPose> idlers;
    idlers.emplace_back(left_assembly->GetIdler()->GetWheelBody()->GetFrame_REF_to_abs());
    idlers.emplace_back(right_assembly->GetIdler()->GetWheelBody()->GetFrame_REF_to_abs());

    std::vector<SynPose> road_wheels;
    for (int i = 0; i < m_tracked_vehicle->GetTrackAssembly(LEFT)->GetNumRoadWheelAssemblies(); i++)
        road_wheels.emplace_back(left_assembly->GetRoadWheel(i)->GetWheelBody()->GetFrame_REF_to_abs());

    for (int i = 0; i < m_tracked_vehicle->GetTrackAssembly(RIGHT)->GetNumRoadWheelAssemblies(); i++)
        road_wheels.emplace_back(right_assembly->GetRoadWheel(i)->GetWheelBody()->GetFrame_REF_to_abs());

    auto time = m_tracked_vehicle->GetSystem()->GetChTime();
    m_state->SetState(time, chassis, track_shoes, sprockets, idlers, road_wheels);
}

// ---------------------------------------------------------------------------

void SynTrackedVehicle::SetZombieVisualizationFiles(std::string chassis_vis_file,
                                                    std::string track_shoe_vis_file,
                                                    std::string left_sprocket_vis_file,
                                                    std::string right_sprocket_vis_file,
                                                    std::string left_idler_vis_file,
                                                    std::string right_idler_vis_file,
                                                    std::string left_road_wheel_vis_file,
                                                    std::string right_road_wheel_vis_file) {
    m_description->m_chassis_vis_file = chassis_vis_file;
    m_description->m_track_shoe_vis_file = track_shoe_vis_file;
    m_description->m_left_sprocket_vis_file = left_sprocket_vis_file;
    m_description->m_right_sprocket_vis_file = right_sprocket_vis_file;
    m_description->m_left_idler_vis_file = left_idler_vis_file;
    m_description->m_right_idler_vis_file = right_idler_vis_file;
    m_description->m_left_road_wheel_vis_file = left_road_wheel_vis_file;
    m_description->m_right_road_wheel_vis_file = right_road_wheel_vis_file;
}

void SynTrackedVehicle::SetNumAssemblyComponents(int num_track_shoes,
                                                 int num_sprockets,
                                                 int num_idlers,
                                                 int num_road_wheels) {
    m_description->m_num_track_shoes = num_track_shoes;
    m_description->m_num_sprockets = num_sprockets;
    m_description->m_num_idlers = num_idlers;
    m_description->m_num_road_wheels = num_road_wheels;
}

// ---------------------------------------------------------------------------

void SynTrackedVehicle::Synchronize(double time, const ChDriver::Inputs& driver_inputs) {
    m_tracked_vehicle->Synchronize(time, driver_inputs, m_shoe_forces_left, m_shoe_forces_right);
}

// ---------------------------------------------------------------------------

rapidjson::Document SynTrackedVehicle::ParseVehicleFileJSON(const std::string& filename) {
    auto d = SynVehicle::ParseVehicleFileJSON(filename);

    // -----------------------------------
    // Further validation of the JSON file
    // -----------------------------------
    assert(d["Vehicle"].HasMember("Track Shoe Visualization Type"));
    assert(d["Vehicle"].HasMember("Sprocket Visualization Type"));
    assert(d["Vehicle"].HasMember("Idler Visualization Type"));
    assert(d["Vehicle"].HasMember("Road Wheel Visualization Type"));
    assert(d["Vehicle"].HasMember("Road Wheel Assembly Visualization Type"));

    assert(d["Zombie"].HasMember("Track Shoe Visualization File"));
    assert(d["Zombie"].HasMember("Left Sprocket Visualization File"));
    assert(d["Zombie"].HasMember("Right Sprocket Visualization File"));
    assert(d["Zombie"].HasMember("Left Idler Visualization File"));
    assert(d["Zombie"].HasMember("Right Idler Visualization File"));
    assert(d["Zombie"].HasMember("Left Road Wheel Visualization File"));
    assert(d["Zombie"].HasMember("Right Road Wheel Visualization File"));

    assert(d["Zombie"].HasMember("Number Of Track Shoes"));
    assert(d["Zombie"].HasMember("Number Of Sprockets"));
    assert(d["Zombie"].HasMember("Number Of Idlers"));
    assert(d["Zombie"].HasMember("Number Of Road Wheels"));

    // Set the zombie visualization files
    m_description->m_chassis_vis_file = d["Zombie"]["Chassis Visualization File"].GetString();
    m_description->m_track_shoe_vis_file = d["Zombie"]["Track Shoe Visualization File"].GetString();
    m_description->m_left_sprocket_vis_file = d["Zombie"]["Left Sprocket Visualization File"].GetString();
    m_description->m_right_sprocket_vis_file = d["Zombie"]["Right Sprocket Visualization File"].GetString();
    m_description->m_left_idler_vis_file = d["Zombie"]["Left Idler Visualization File"].GetString();
    m_description->m_right_idler_vis_file = d["Zombie"]["Right Idler Visualization File"].GetString();
    m_description->m_left_road_wheel_vis_file = d["Zombie"]["Left Road Wheel Visualization File"].GetString();
    m_description->m_right_road_wheel_vis_file = d["Zombie"]["Right Road Wheel Visualization File"].GetString();

    // Set number of assembly components
    m_description->m_num_track_shoes = d["Zombie"]["Number Of Track Shoes"].GetInt();
    m_description->m_num_sprockets = d["Zombie"]["Number Of Sprockets"].GetInt();
    m_description->m_num_idlers = d["Zombie"]["Number Of Idlers"].GetInt();
    m_description->m_num_road_wheels = d["Zombie"]["Number Of Road Wheels"].GetInt();

    return d;
}

void SynTrackedVehicle::CreateVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system) {
    // Parse file
    auto d = ParseVehicleFileJSON(filename);

    // Create the vehicle system
    std::string vehicle_filename = d["Vehicle"]["Input File"].GetString();
    m_tracked_vehicle = new TrackedVehicle(system, vehicle::GetDataFile(vehicle_filename));

    m_tracked_vehicle->Initialize(coord_sys);

    m_shoe_forces_left = TerrainForces(m_tracked_vehicle->GetNumTrackShoes(LEFT));
    m_shoe_forces_right = TerrainForces(m_tracked_vehicle->GetNumTrackShoes(RIGHT));

    // Set vehicle visualization types
    m_tracked_vehicle->SetChassisVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Chassis Visualization Type"].GetString()));
    m_tracked_vehicle->SetTrackShoeVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Track Shoe Visualization Type"].GetString()));
    m_tracked_vehicle->SetSprocketVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Sprocket Visualization Type"].GetString()));
    m_tracked_vehicle->SetIdlerVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Idler Visualization Type"].GetString()));
    m_tracked_vehicle->SetRoadWheelVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Road Wheel Visualization Type"].GetString()));
    m_tracked_vehicle->SetRoadWheelAssemblyVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Road Wheel Assembly Visualization Type"].GetString()));

    // Create and initialize the powertrain system
    std::string powertrain_filename = d["Powertrain"]["Input File"].GetString();
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(powertrain_filename));
    m_tracked_vehicle->InitializePowertrain(powertrain);

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------
    auto solver = chrono_types::make_shared<ChSolverBB>();
    solver->SetMaxIterations(120);
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    m_system->SetSolver(solver);

    m_system->SetMaxPenetrationRecoverySpeed(1.5);
    m_system->SetMinBounceSpeed(2.0);
}

// ---------------------------------------------------------------------------

void SynTrackedVehicle::AddMeshToVector(std::shared_ptr<ChTriangleMeshShape> trimesh,
                                        std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                                        ChSystem* system) {
    for (auto& ref : ref_list) {
        ref = chrono_types::make_shared<ChBodyAuxRef>();
        ref->AddAsset(trimesh);
        ref->SetCollide(false);
        ref->SetBodyFixed(true);
        system->Add(ref);
    }
}

void SynTrackedVehicle::AddMeshToVector(std::shared_ptr<ChTriangleMeshShape> left,
                                        std::shared_ptr<ChTriangleMeshShape> right,
                                        std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                                        ChSystem* system) {
    for (int i = 0; i < ref_list.size() / 2; i++) {
        auto& ref_left = ref_list[i];
        ref_left = chrono_types::make_shared<ChBodyAuxRef>();
        ref_left->AddAsset(left);
        ref_left->SetCollide(false);
        ref_left->SetBodyFixed(true);
        system->Add(ref_left);

        auto& ref_right = ref_list[ref_list.size() - i - 1];
        ref_right = chrono_types::make_shared<ChBodyAuxRef>();
        ref_right->AddAsset(right);
        ref_right->SetCollide(false);
        ref_right->SetBodyFixed(true);
        system->Add(ref_right);
    }
}

}  // namespace synchrono
}  // namespace chrono
