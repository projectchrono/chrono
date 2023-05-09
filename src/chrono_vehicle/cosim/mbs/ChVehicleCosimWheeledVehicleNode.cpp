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
// Wheeled vehicle system co-simulated with tire nodes and a terrain node.
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
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimWheeledVehicleNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

class WheeledVehicleDBPDriver : public ChDriver {
  public:
    WheeledVehicleDBPDriver(std::shared_ptr<ChWheeledVehicle> vehicle, std::shared_ptr<ChFunction> dbp_mot_func)
        : ChDriver(*vehicle), m_wheeled_vehicle(vehicle), m_func(dbp_mot_func) {}

    virtual void Synchronize(double time) override {
        m_steering = 0;
        m_braking = 0;

        double ang_speed = m_func->Get_y(time);

        for (auto& axle : m_wheeled_vehicle->GetAxles()) {
            axle->m_suspension->GetAxle(VehicleSide::LEFT)->SetPos_dt(ang_speed);
            axle->m_suspension->GetAxle(VehicleSide::RIGHT)->SetPos_dt(ang_speed);
        }
    }

    std::shared_ptr<ChWheeledVehicle> m_wheeled_vehicle;
    std::shared_ptr<ChFunction> m_func;
};

// -----------------------------------------------------------------------------

ChVehicleCosimWheeledVehicleNode::ChVehicleCosimWheeledVehicleNode(const std::string& vehicle_json,
                                                                   const std::string& engine_json,
                                                                   const std::string& transmission_json)
    : ChVehicleCosimWheeledMBSNode(), m_num_spindles(0), m_init_yaw(0), m_chassis_fixed(false) {
    m_vehicle = chrono_types::make_shared<WheeledVehicle>(m_system, vehicle_json);
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    m_powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    m_terrain = chrono_types::make_shared<ChTerrain>();
}

ChVehicleCosimWheeledVehicleNode::ChVehicleCosimWheeledVehicleNode(std::shared_ptr<ChWheeledVehicle> vehicle,
                                                                   std::shared_ptr<ChPowertrainAssembly> powertrain)
    : ChVehicleCosimWheeledMBSNode(), m_num_spindles(0), m_init_yaw(0), m_chassis_fixed(false) {
    // Ensure the vehicle system has a null ChSystem
    if (vehicle->GetSystem())
        return;

    m_vehicle = vehicle;
    m_powertrain = powertrain;
    m_terrain = chrono_types::make_shared<ChTerrain>();

    // Associate the vehicle system with this node's ChSystem
    m_vehicle->SetSystem(m_system);
}

ChVehicleCosimWheeledVehicleNode::~ChVehicleCosimWheeledVehicleNode() {}

// -----------------------------------------------------------------------------

void ChVehicleCosimWheeledVehicleNode::InitializeMBS(const std::vector<ChVector<>>& tire_info,
                                                     const ChVector2<>& terrain_size,
                                                     double terrain_height) {
    // Initialize vehicle
    ChCoordsys<> init_pos(m_init_loc + ChVector<>(0, 0, terrain_height), Q_from_AngZ(m_init_yaw));

    m_vehicle->Initialize(init_pos);
    m_vehicle->GetChassis()->SetFixed(m_chassis_fixed);
    m_vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    m_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Initialize powertrain
    m_vehicle->InitializePowertrain(m_powertrain);

    // Create and initialize the dummy tires
    int itire = 0;
    for (auto& axle : m_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = chrono_types::make_shared<DummyTire>(itire, tire_info[itire].x(), tire_info[itire].y(),
                                                             tire_info[itire].z());
            m_vehicle->InitializeTire(tire, wheel, VisualizationType::NONE);
            m_tires.push_back(tire);
            itire++;
        }
    }

    // Extract and cache spindle bodies
    auto num_axles = m_vehicle->GetNumberAxles();

    m_num_spindles = 2 * num_axles;
    assert(m_num_spindles == (int)m_num_tire_nodes);

    auto total_mass = m_vehicle->GetMass();
    for (int is = 0; is < m_num_spindles; is++) {
        auto tire_mass = tire_info[is].x();
        m_spindle_loads.push_back(tire_mass + total_mass / m_num_spindles);
    }
}

// -----------------------------------------------------------------------------
int ChVehicleCosimWheeledVehicleNode::GetNumSpindles() const {
    return m_num_spindles;
}

std::shared_ptr<ChBody> ChVehicleCosimWheeledVehicleNode::GetSpindleBody(unsigned int i) const {
    VehicleSide side = (i % 2 == 0) ? VehicleSide::LEFT : VehicleSide::RIGHT;
    return m_vehicle->GetWheel(i / 2, side)->GetSpindle();
}

double ChVehicleCosimWheeledVehicleNode::GetSpindleLoad(unsigned int i) const {
    return m_spindle_loads[i];
}

BodyState ChVehicleCosimWheeledVehicleNode::GetSpindleState(unsigned int i) const {
    VehicleSide side = (i % 2 == 0) ? VehicleSide::LEFT : VehicleSide::RIGHT;
    auto spindle_body = m_vehicle->GetWheel(i / 2, side)->GetSpindle();
    BodyState state;

    state.pos = spindle_body->GetPos();
    state.rot = spindle_body->GetRot();
    state.lin_vel = spindle_body->GetPos_dt();
    state.ang_vel = spindle_body->GetWvel_par();

    return state;
}

std::shared_ptr<ChBody> ChVehicleCosimWheeledVehicleNode::GetChassisBody() const {
    return m_vehicle->GetChassisBody();
}

void ChVehicleCosimWheeledVehicleNode::OnInitializeDBPRig(std::shared_ptr<ChFunction> func) {
    // Disconnect the driveline
    m_vehicle->DisconnectDriveline();
    // Overwrite any driver attached to the vehicle with a custom driver which imposes zero steering and braking and
    // directly sets the angular speed of the vehicle axleshafts as returned by the provided motor function.
    SetDriver(chrono_types::make_shared<WheeledVehicleDBPDriver>(m_vehicle, func));
}

// -----------------------------------------------------------------------------

void ChVehicleCosimWheeledVehicleNode::PreAdvance() {
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
    m_vehicle->Synchronize(time, driver_inputs, *m_terrain);
}

void ChVehicleCosimWheeledVehicleNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    // Cache the spindle force on the corresponding dummy tire. This force will be applied to the associated ChWheel on
    // synchronization of the vehicle system (in PreAdvance)
    m_tires[i]->m_force = spindle_force;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimWheeledVehicleNode::OnOutputData(int frame) {
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

void ChVehicleCosimWheeledVehicleNode::WriteBodyInformation(utils::CSV_writer& csv) {
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
}

}  // end namespace vehicle
}  // end namespace chrono
