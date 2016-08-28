// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// Definition of the VEHICLE NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <omp.h>
#include <algorithm>
#include <set>
#include <vector>
#include "mpi.h"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "VehicleNode.h"

using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

class MyDriver : public ChDriver {
public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            m_throttle = 0.8;
        else
            m_throttle = 4 * eff_time;
    }

private:
    double m_delay;
};

// =============================================================================

// -----------------------------------------------------------------------------
// Construction of the vehicle node:
// - create the (sequential) Chrono system and set solver parameters
// -----------------------------------------------------------------------------
VehicleNode::VehicleNode() : BaseNode("VEHICLE"), m_vehicle(nullptr), m_powertrain(nullptr), m_driver(nullptr) {
    m_prefix = "[Vehicle node]";

    cout << m_prefix << " num_threads = 1" << endl;

    // ------------------------
    // Default model parameters
    // ------------------------

    m_delay = 0.5;
    m_chassis_fixed = false;

    // ----------------------------------
    // Create the (sequential) DEM system
    // ----------------------------------

    m_system = new ChSystemDEM;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number threads
    m_system->SetParallelThreadNumber(1);
    CHOMPfunctions::SetNumThreads(1);

    // Solver settings
    m_system->SetMaxItersSolverSpeed(100);
    m_system->SetMaxItersSolverStab(100);
    m_system->SetSolverType(ChSystem::SOLVER_SOR);
    m_system->SetTol(1e-10);
    m_system->SetTolForce(1e-8);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
VehicleNode::~VehicleNode() {
    delete m_vehicle;
    delete m_powertrain;
    delete m_driver;
    delete m_system;
}

// -----------------------------------------------------------------------------
// Initialization of the vehicle node:
// - receive terrain height and container half-length
// - construct and initialize subsystems
// - send initial wheel states to tire nodes
// -----------------------------------------------------------------------------
void VehicleNode::Initialize() {
    // ----------------------------------
    // Receive terrain height information
    // ----------------------------------

    // Receive terrain height and half-length of the container
    double init_dim[2];
    MPI_Status status;
    MPI_Recv(init_dim, 2, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    cout << m_prefix << " Received initial terrain height = " << init_dim[0] << endl;
    cout << m_prefix << " Received container half-length = " << init_dim[1] << endl;

    // Set initial vehicle position and orientation
    ChVector<> init_loc(2.75 - init_dim[1], 0, 0.6 + init_dim[0]);
    ChQuaternion<> init_rot(1, 0, 0, 0);

    // --------------------------------------------
    // Create and initialize subsystems
    // --------------------------------------------

    m_vehicle = new HMMWV_Vehicle(m_system, m_chassis_fixed);
    m_vehicle->Initialize(ChCoordsys<>(init_loc, init_rot));

    m_powertrain = new HMMWV_Powertrain;
    m_powertrain->Initialize(m_vehicle->GetChassis(), m_vehicle->GetDriveshaft());

    m_driver = new MyDriver(*m_vehicle, m_delay);
    m_driver->Initialize();

    // ---------------------------------------
    // Send wheel information to tire nodes
    // ---------------------------------------

    m_num_wheels = 2 * m_vehicle->GetNumberAxles();
    m_tire_forces.resize(m_num_wheels);

    // Send wheel initial states.
    // It is assumed that initial linear and angular velocity are always 0.
    double bufWS[7];
    for (int iw = 0; iw < m_num_wheels; iw++) {
        WheelState wheel_state = m_vehicle->GetWheelState(WheelID(iw));
        bufWS[0] = wheel_state.pos.x;
        bufWS[1] = wheel_state.pos.y;
        bufWS[2] = wheel_state.pos.z;
        bufWS[3] = wheel_state.rot.e0;
        bufWS[4] = wheel_state.rot.e1;
        bufWS[5] = wheel_state.rot.e2;
        bufWS[6] = wheel_state.rot.e3;
        MPI_Send(bufWS, 7, MPI_DOUBLE, TIRE_NODE_RANK(iw), iw, MPI_COMM_WORLD);
    }

    // -------------------------------------
    // Write file with vehicle node settings
    // -------------------------------------

    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);

    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
}

// -----------------------------------------------------------------------------
// Synchronization of the vehicle node:
// - receive forces from tire nodes (used in vehicle synchronization)
// - send current wheel states to tire nodes
// - synchronize vehicle, powertrain, driver
// -----------------------------------------------------------------------------
void VehicleNode::Synchronize(int step_number, double time) {
    //// TODO check this

    // Get current driver outputs
    double steering = m_driver->GetSteering();
    double throttle = m_driver->GetThrottle();
    double braking = m_driver->GetBraking();

    // Get current driveshaft speed and powertrain output torque
    double driveshaft_speed = m_vehicle->GetDriveshaftSpeed();
    double powertrain_torque = m_powertrain->GetOutputTorque();

    // Receive tire forces from each of the tire nodes
    double bufTF[9];
    MPI_Status statusTF;
    for (int iw = 0; iw < m_num_wheels; iw++) {
        MPI_Recv(bufTF, 9, MPI_DOUBLE, TIRE_NODE_RANK(iw), iw, MPI_COMM_WORLD, &statusTF);
        m_tire_forces[iw].force = ChVector<>(bufTF[0], bufTF[1], bufTF[2]);
        m_tire_forces[iw].moment = ChVector<>(bufTF[3], bufTF[4], bufTF[5]);
        m_tire_forces[iw].point = ChVector<>(bufTF[6], bufTF[7], bufTF[8]);
    }

    // Send complete wheel states to each of the tire nodes
    double bufWS[14];
    for (int iw = 0; iw < m_num_wheels; iw++) {
        WheelState wheel_state = m_vehicle->GetWheelState(WheelID(iw));
        bufWS[0] = wheel_state.pos.x;
        bufWS[1] = wheel_state.pos.y;
        bufWS[2] = wheel_state.pos.z;
        bufWS[3] = wheel_state.rot.e0;
        bufWS[4] = wheel_state.rot.e1;
        bufWS[5] = wheel_state.rot.e2;
        bufWS[6] = wheel_state.rot.e3;
        bufWS[7] = wheel_state.lin_vel.x;
        bufWS[8] = wheel_state.lin_vel.y;
        bufWS[9] = wheel_state.lin_vel.z;
        bufWS[10] = wheel_state.ang_vel.x;
        bufWS[11] = wheel_state.ang_vel.y;
        bufWS[12] = wheel_state.ang_vel.z;
        bufWS[13] = wheel_state.omega;
        MPI_Send(bufWS, 14, MPI_DOUBLE, TIRE_NODE_RANK(iw), iw, MPI_COMM_WORLD);
    }

    // Synchronize vehicle, powertrain, and driver
    m_vehicle->Synchronize(time, steering, braking, powertrain_torque, m_tire_forces);
    m_powertrain->Synchronize(time, throttle, driveshaft_speed);
    m_driver->Synchronize(time);
}

// -----------------------------------------------------------------------------
// Advance simulation of the vehicle node by the specified duration
// -----------------------------------------------------------------------------
void VehicleNode::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void VehicleNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        // Position, orientation, velocity of the chassis reference frame
        // (relative to absolute frame)
        const ChVector<>& pos = m_vehicle->GetChassisPos();
        const ChQuaternion<>& rot = m_vehicle->GetChassisRot();
        const ChVector<>& lin_vel = m_vehicle->GetChassis()->GetFrame_REF_to_abs().GetPos_dt();
        const ChVector<>& ang_vel = m_vehicle->GetChassis()->GetFrame_REF_to_abs().GetWvel_par();

        // Current driver inputs
        double steering = m_driver->GetSteering();
        double throttle = m_driver->GetThrottle();
        double braking = m_driver->GetBraking();

        m_outf << m_system->GetChTime() << del;
        m_outf << steering << del << throttle << del << braking << del;
        m_outf << pos.x << del << pos.y << del << pos.z << del;
        m_outf << rot.e0 << del << rot.e1 << del << rot.e2 << del << rot.e3 << del;
        m_outf << lin_vel.x << del << lin_vel.y << del << lin_vel.z << del;
        m_outf << ang_vel.x << del << ang_vel.y << del << ang_vel.z << del;
        m_outf << endl;
    }

    /*
    // Create and write frame output file.
    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", m_node_out_dir.c_str(), frame + 1);

    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;  // current time
    WriteStateInformation(csv);            // 
    csv.write_to_file(filename);

    cout << m_prefix << " write output file ==> " << filename << endl;
    */
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void VehicleNode::WriteStateInformation(utils::CSV_writer& csv) {
    //// TODO
}
