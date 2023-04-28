// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Tracked vehicle system co-simulated with track nodes and a terrain node.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <fstream>
#include <algorithm>
#include <set>
#include <vector>

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimTrackedVehicleNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

class TrackedVehicleDBPDriver : public ChDriver {
  public:
    TrackedVehicleDBPDriver(std::shared_ptr<ChTrackedVehicle> vehicle, std::shared_ptr<ChFunction> dbp_mot_func)
        : ChDriver(*vehicle), m_vehicle(vehicle), m_func(dbp_mot_func) {}

    virtual void Synchronize(double time) override {
        m_steering = 0;
        m_braking = 0;

        double ang_speed = m_func->Get_y(time);
        m_vehicle->GetTrackAssembly(VehicleSide::LEFT)->GetSprocket()->GetAxle()->SetPos_dt(ang_speed);
        m_vehicle->GetTrackAssembly(VehicleSide::RIGHT)->GetSprocket()->GetAxle()->SetPos_dt(ang_speed);
    }

    std::shared_ptr<ChTrackedVehicle> m_vehicle;
    std::shared_ptr<ChFunction> m_func;
};

// -----------------------------------------------------------------------------

ChVehicleCosimTrackedVehicleNode::ChVehicleCosimTrackedVehicleNode(const std::string& vehicle_json,
                                                                   const std::string& engine_json,
                                                                   const std::string& transmission_json)
    : ChVehicleCosimTrackedMBSNode(), m_init_yaw(0), m_chassis_fixed(false) {
    m_vehicle = chrono_types::make_shared<TrackedVehicle>(m_system, vehicle_json);
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    m_powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
}

ChVehicleCosimTrackedVehicleNode::ChVehicleCosimTrackedVehicleNode(std::shared_ptr<ChTrackedVehicle> vehicle,
                                                                   std::shared_ptr<ChPowertrainAssembly> powertrain)
    : ChVehicleCosimTrackedMBSNode(), m_init_yaw(0), m_chassis_fixed(false) {
    // Ensure the vehicle system has a null ChSystem
    if (vehicle->GetSystem())
        return;

    m_vehicle = vehicle;
    m_powertrain = powertrain;

    // Associate the vehicle system with this node's ChSystem
    m_vehicle->SetSystem(m_system);
}

ChVehicleCosimTrackedVehicleNode::~ChVehicleCosimTrackedVehicleNode() {}

// -----------------------------------------------------------------------------

void ChVehicleCosimTrackedVehicleNode::InitializeMBS(const ChVector2<>& terrain_size, double terrain_height) {
    // Initialize vehicle
    ChCoordsys<> init_pos(m_init_loc + ChVector<>(0, 0, terrain_height), Q_from_AngZ(m_init_yaw));

    m_vehicle->Initialize(init_pos);
    m_vehicle->GetChassis()->SetFixed(m_chassis_fixed);
    m_vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    m_vehicle->SetSprocketVisualizationType(VisualizationType::MESH);
    m_vehicle->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetIdlerWheelVisualizationType(VisualizationType::MESH);
    m_vehicle->SetRoadWheelVisualizationType(VisualizationType::MESH);
    m_vehicle->SetRollerVisualizationType(VisualizationType::MESH);
    m_vehicle->SetTrackShoeVisualizationType(VisualizationType::MESH);

    // Initialize powertrain
    m_vehicle->InitializePowertrain(m_powertrain);

    // Size vectors of track shoe forces
    m_shoe_forces[0].resize(m_vehicle->GetNumTrackShoes(VehicleSide::LEFT));
    m_shoe_forces[1].resize(m_vehicle->GetNumTrackShoes(VehicleSide::RIGHT));
}

// -----------------------------------------------------------------------------

ChVehicleGeometry ChVehicleCosimTrackedVehicleNode::GetTrackShoeContactGeometry() const {
    return m_vehicle->GetTrackShoe(VehicleSide::LEFT, 0)->GetGroundContactGeometry();
}

double ChVehicleCosimTrackedVehicleNode::GetTrackShoeMass() const {
    return m_vehicle->GetTrackShoe(VehicleSide::LEFT, 0)->GetMass();
}

// -----------------------------------------------------------------------------

int ChVehicleCosimTrackedVehicleNode::GetNumTracks() const {
    return 2;
}

size_t ChVehicleCosimTrackedVehicleNode::GetNumTrackShoes(int track_id) const {
    return m_vehicle->GetNumTrackShoes(track_id == 0 ? VehicleSide::LEFT : VehicleSide::RIGHT);
}

size_t ChVehicleCosimTrackedVehicleNode::GetNumTrackShoes() const {
    return GetNumTrackShoes(VehicleSide::LEFT) + GetNumTrackShoes(VehicleSide::RIGHT);
}

std::shared_ptr<ChBody> ChVehicleCosimTrackedVehicleNode::GetTrackShoeBody(int track_id, int shoe_id) const {
    return m_vehicle->GetTrackShoe(track_id == 0 ? VehicleSide::LEFT : VehicleSide::RIGHT, shoe_id)->GetShoeBody();
}

BodyState ChVehicleCosimTrackedVehicleNode::GetTrackShoeState(int track_id, int shoe_id) const {
    return m_vehicle->GetTrackShoeState(track_id == 0 ? VehicleSide::LEFT : VehicleSide::RIGHT, shoe_id);
}

std::shared_ptr<ChBody> ChVehicleCosimTrackedVehicleNode::GetChassisBody() const {
    return m_vehicle->GetChassisBody();
}

double ChVehicleCosimTrackedVehicleNode::GetSprocketAddendumRadius() const {
    return m_vehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetAddendumRadius();
}

void ChVehicleCosimTrackedVehicleNode::OnInitializeDBPRig(std::shared_ptr<ChFunction> func) {
    // Disconnect the driveline
    m_vehicle->DisconnectDriveline();

    // Overwrite any driver attached to the vehicle with a custom driver which imposes zero steering and braking and
    // directly sets the angular speed of the sprockets as returned by the provided motor function.
    SetDriver(chrono_types::make_shared<TrackedVehicleDBPDriver>(m_vehicle, func));
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTrackedVehicleNode::PreAdvance() {
    // Synchronize vehicle systems
    double time = m_vehicle->GetChTime();
    DriverInputs driver_inputs;
    if (m_driver) {
        driver_inputs = m_driver->GetInputs();
        m_driver->Synchronize(time);
    } else {
        driver_inputs.m_steering = 0;
        driver_inputs.m_throttle = 0.5;
        driver_inputs.m_braking = 0;
    }
    m_vehicle->Synchronize(time, driver_inputs, m_shoe_forces[0], m_shoe_forces[1]);
}

void ChVehicleCosimTrackedVehicleNode::ApplyTrackShoeForce(int track_id, int shoe_id, const TerrainForce& force) {
    // Cache the track shoe force.
    // Forces acting on all track shoes will be applied during synchronization of the vehicle system (in PreAdvance)
    m_shoe_forces[track_id][shoe_id] = force;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTrackedVehicleNode::OnOutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& pos = m_vehicle->GetPos();

        m_outf << m_system->GetChTime() << del;
        // Body states
        m_outf << pos.x() << del << pos.y() << del << pos.z() << del;
        // Solver statistics (for last integration step)
        m_outf << m_system->GetTimerStep() << del << m_system->GetTimerLSsetup() << del << m_system->GetTimerLSsolve()
               << del << m_system->GetTimerUpdate() << del;
        if (m_int_type == ChTimestepper::Type::HHT) {
            m_outf << m_integrator->GetNumIterations() << del << m_integrator->GetNumSetupCalls() << del
                   << m_integrator->GetNumSolveCalls() << del;
        }
        m_outf << endl;
    }

    // Create and write frame output file.
    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;  // current time
    WriteBodyInformation(csv);             // vehicle body states

    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Vehicle node] write output file ==> " << filename << endl;
}

void ChVehicleCosimTrackedVehicleNode::WriteBodyInformation(utils::CSV_writer& csv) {
    //// RADU TODO
    /*

        // Write number of bodies
        csv << 1 + m_num_spindles << endl;

        // Write body state information
        auto chassis = m_vehicle->GetChassisBody();
        csv << chassis->GetPos() << chassis->GetRot() << chassis->GetPos_dt() << chassis->GetRot_dt() << endl;

        for (auto& axle : m_vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto spindle_body = wheel->GetSpindle();
                csv << spindle_body->GetPos() << spindle_body->GetRot() << spindle_body->GetPos_dt()
                    << spindle_body->GetRot_dt() << endl;
            }
        }
    */
}

}  // end namespace vehicle
}  // end namespace chrono
