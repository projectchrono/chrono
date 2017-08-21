// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Cosimulation node responsible for simulating a wheeled vehicle system.
//
// =============================================================================

#include <algorithm>
#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimManager.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimVehicleNode.h"

namespace chrono {
namespace vehicle {

ChCosimVehicleNode::ChCosimVehicleNode(int rank, ChWheeledVehicle* vehicle, ChPowertrain* powertrain, ChDriver* driver)
    : ChCosimNode(rank, m_vehicle->GetSystem()), m_vehicle(vehicle), m_powertrain(powertrain), m_driver(driver) {
    m_num_wheels = 2 * m_vehicle->GetNumberAxles();
    m_tire_forces.resize(m_num_wheels);
}

void ChCosimVehicleNode::SetStepsize(double stepsize) {
    ChCosimNode::SetStepsize(stepsize);
    m_vehicle->SetStepsize(stepsize);
}

void ChCosimVehicleNode::Initialize(const ChCoordsys<>& chassisPos) {
    m_vehicle->Initialize(chassisPos);
    m_powertrain->Initialize(m_vehicle->GetChassisBody(), m_vehicle->GetDriveshaft());
    m_driver->Initialize();

    // Send wheel masses and inertias to the tire nodes
    double props[4];
    for (int iw = 0; iw < m_num_wheels; iw++) {
        double mass = m_vehicle->GetWheelBody(WheelID(iw))->GetMass();
        ChVector<> inertia = m_vehicle->GetWheelBody(WheelID(iw))->GetInertiaXX();
        props[0] = mass;
        props[1] = inertia.x;
        props[2] = inertia.y;
        props[3] = inertia.z;
        MPI_Send(props, 4, MPI_DOUBLE, TIRE_NODE_RANK(iw), iw, MPI_COMM_WORLD);
        if (m_verbose) {
            printf("Vehicle node %d.  Send to %d props = %g %g %g %g\n", m_rank, TIRE_NODE_RANK(iw), props[0], props[1],
                   props[2], props[3]);
        }
    }
}

void ChCosimVehicleNode::Synchronize(double time) {
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

    // Send wheel states to each of the tire nodes
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

void ChCosimVehicleNode::Advance(double step) {
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_powertrain->Advance(step);
    m_driver->Advance(step);
}

}  // end namespace vehicle
}  // end namespace chrono
