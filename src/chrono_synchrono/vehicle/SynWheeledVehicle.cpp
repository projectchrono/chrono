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
// Wrapper class for ChWheeledVehicles. Additional functions here are related to
// initializing this as a zombie (setting visual representations, # of wheels)
//
// =============================================================================

#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"

#include "chrono/core/ChLog.h"

// MSVC seems to need these otherwise it compiles out the templated customWheeledVehicle
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_models/vehicle/gator/Gator.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono::vehicle;
using namespace rapidjson;

namespace chrono {
namespace synchrono {

template class SynCustomWheeledVehicle<class chrono::vehicle::hmmwv::HMMWV_Full>;
template class SynCustomWheeledVehicle<class chrono::vehicle::gator::Gator>;

SynWheeledVehicle::SynWheeledVehicle() : SynVehicle() {
    m_state = chrono_types::make_shared<SynWheeledVehicleState>();
    m_description = chrono_types::make_shared<SynWheeledVehicleDescription>();
}

SynWheeledVehicle::SynWheeledVehicle(ChWheeledVehicle* wheeled_vehicle)
    : SynVehicle(false), m_wheeled_vehicle(wheeled_vehicle) {
    if (!wheeled_vehicle)
        throw ChException("SynWheeledVehicle constructor - wheeled_vehicle is NULL");
    m_system = m_wheeled_vehicle->GetSystem();

    m_state = chrono_types::make_shared<SynWheeledVehicleState>();
    m_description = chrono_types::make_shared<SynWheeledVehicleDescription>();
}

SynWheeledVehicle::SynWheeledVehicle(const ChCoordsys<>& coord_sys,
                                     const std::string& filename,
                                     ChContactMethod contact_method)
    : SynVehicle(filename, contact_method) {
    m_state = chrono_types::make_shared<SynWheeledVehicleState>();
    m_description = chrono_types::make_shared<SynWheeledVehicleDescription>();

    CreateVehicle(coord_sys, filename, m_system);
}

SynWheeledVehicle::SynWheeledVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system)
    : SynVehicle(filename, system) {
    m_state = chrono_types::make_shared<SynWheeledVehicleState>();
    m_description = chrono_types::make_shared<SynWheeledVehicleDescription>();

    CreateVehicle(coord_sys, filename, m_system);
}

SynWheeledVehicle::SynWheeledVehicle(const std::string& filename) : SynVehicle(filename) {
    m_state = chrono_types::make_shared<SynWheeledVehicleState>();
    m_description = chrono_types::make_shared<SynWheeledVehicleDescription>();

    CreateZombie(filename);
}

SynWheeledVehicle::~SynWheeledVehicle() {
    if (m_wheeled_vehicle && m_owns_vehicle) {
        delete m_wheeled_vehicle;

        if (m_system)
            delete m_system;
    }
}

void SynWheeledVehicle::InitializeZombie(ChSystem* system) {
    CreateChassisZombieBody(m_description->m_chassis_vis_file, system);

    // For each wheel, create a tire mesh and wheel mesh.
    // If it is a right side wheel, a 180 degree rotation is made

    for (int i = 0; i < m_description->m_num_wheels; i++) {
        auto tire_trimesh = CreateMeshZombieComponent(m_description->m_tire_vis_file);
        auto wheel_trimesh = CreateMeshZombieComponent(m_description->m_wheel_vis_file);

        ChQuaternion<> rot = (i + 1 % 2 == 0) ? Q_from_AngZ(0) : Q_from_AngZ(CH_C_PI);
        wheel_trimesh->GetMesh()->Transform(ChVector<>(), ChMatrix33<>(rot));
        tire_trimesh->GetMesh()->Transform(ChVector<>(), ChMatrix33<>(rot));

        auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
        wheel->AddAsset(wheel_trimesh);
        wheel->AddAsset(tire_trimesh);
        wheel->SetCollide(false);
        wheel->SetBodyFixed(true);
        system->Add(wheel);

        m_wheel_list.push_back(wheel);
    }

    m_system = system;
}

void SynWheeledVehicle::SynchronizeZombie(SynMessage* message) {
    if (m_zombie_body == nullptr)
        return;

    if (message != nullptr) {
        if (message->GetType() != SynMessageType::WHEELED_VEHICLE)
            return;

        m_state = ((SynWheeledVehicleMessage*)message)->GetWheeledState();
    } else {
        // Dead reckon if state was not received
        // m_state->chassis.Step(HEARTBEAT);
        // for (auto& wheel : m_state->wheels)
        //     wheel.Step(HEARTBEAT);
    }

    m_zombie_body->SetFrame_REF_to_abs(m_state->chassis.GetFrame());
    for (int i = 0; i < m_state->wheels.size(); i++)
        m_wheel_list[i]->SetFrame_REF_to_abs(m_state->wheels[i].GetFrame());
}

void SynWheeledVehicle::Update() {
    SynPose chassis(m_wheeled_vehicle->GetChassisBody()->GetFrame_REF_to_abs());

    std::vector<SynPose> wheels;
    for (auto axle : m_wheeled_vehicle->GetAxles()) {
        for (auto wheel : axle->GetWheels()) {
            auto wheel_abs = wheel->GetSpindle()->GetFrame_REF_to_abs();
            SynPose frame(wheel_abs.GetPos(), wheel_abs.GetRot());
            frame.GetFrame().SetPos_dt(wheel_abs.GetPos_dt());
            wheels.emplace_back(frame);
        }
    }

    m_state->SetState(m_system->GetChTime(), chassis, wheels);
}

// ---------------------------------------------------------------------------

void SynWheeledVehicle::SetZombieVisualizationFiles(std::string chassis_vis_file,
                                                    std::string wheel_vis_file,
                                                    std::string tire_vis_file) {
    m_description->m_chassis_vis_file = chassis_vis_file;
    m_description->m_wheel_vis_file = wheel_vis_file;
    m_description->m_tire_vis_file = tire_vis_file;
}

// ---------------------------------------------------------------------------

void SynWheeledVehicle::Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain) {
    m_wheeled_vehicle->Synchronize(time, driver_inputs, terrain);
}

// ---------------------------------------------------------------------------

rapidjson::Document SynWheeledVehicle::ParseVehicleFileJSON(const std::string& filename) {
    auto d = SynVehicle::ParseVehicleFileJSON(filename);

    // -----------------------------------
    // Further validation of the JSON file
    // -----------------------------------

    assert(d.HasMember("Tire"));

    assert(d["Vehicle"].HasMember("Suspension Visualization Type"));
    assert(d["Vehicle"].HasMember("Steering Visualization Type"));
    assert(d["Vehicle"].HasMember("Wheel Visualization Type"));

    assert(d["Zombie"].HasMember("Wheel Visualization File"));
    assert(d["Zombie"].HasMember("Tire Visualization File"));
    assert(d["Zombie"].HasMember("Number Of Wheels"));

    // Set the zombie visualization files
    m_description->m_chassis_vis_file = d["Zombie"]["Chassis Visualization File"].GetString();
    m_description->m_wheel_vis_file = d["Zombie"]["Wheel Visualization File"].GetString();
    m_description->m_tire_vis_file = d["Zombie"]["Tire Visualization File"].GetString();

    // Set number of wheels
    m_description->m_num_wheels = d["Zombie"]["Number Of Wheels"].GetInt();

    return d;
}

void SynWheeledVehicle::CreateVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system) {
    // Parse file
    auto d = ParseVehicleFileJSON(filename);

    // Create the vehicle system
    std::string vehicle_filename = d["Vehicle"]["Input File"].GetString();
    m_wheeled_vehicle = new WheeledVehicle(system, vehicle::GetDataFile(vehicle_filename));

    // Initialize the vehicle
    m_wheeled_vehicle->Initialize(coord_sys);

    // Set vehicle visualization types
    m_wheeled_vehicle->SetChassisVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Chassis Visualization Type"].GetString()));
    m_wheeled_vehicle->SetSuspensionVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Suspension Visualization Type"].GetString()));
    m_wheeled_vehicle->SetSteeringVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Steering Visualization Type"].GetString()));
    m_wheeled_vehicle->SetWheelVisualizationType(
        ReadVisualizationTypeJSON(d["Vehicle"]["Wheel Visualization Type"].GetString()));

    // Create and initialize the powertrain system
    std::string powertrain_filename = d["Powertrain"]["Input File"].GetString();
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(powertrain_filename));
    m_wheeled_vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    std::string tire_filename = d["Tire"]["Input File"].GetString();
    VisualizationType tire_visualization = ReadVisualizationTypeJSON(d["Tire"]["Visualization Type"].GetString());
    for (auto& axle : m_wheeled_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_filename));
            m_wheeled_vehicle->InitializeTire(tire, wheel, tire_visualization);
        }
    }
}

}  // namespace synchrono
}  // namespace chrono
