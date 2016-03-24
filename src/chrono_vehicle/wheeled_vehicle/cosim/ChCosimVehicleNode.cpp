// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Cosimulation node responsible for simulating a wheeled vehicle system.
//
// =============================================================================

#include <algorithm>
#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimManager.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimVehicleNode.h"

namespace chrono {
namespace vehicle {

ChCosimVehicleNode::ChCosimVehicleNode(ChWheeledVehicle* vehicle, ChPowertrain* powertrain, ChDriver* driver)
    : m_vehicle(vehicle), m_powertrain(powertrain), m_driver(driver) {
    m_num_wheels = 2 * m_vehicle->GetNumberAxles();
    m_wheel_states.resize(m_num_wheels);
    m_tire_forces.resize(m_num_wheels);
}

void ChCosimVehicleNode::SetStepsize(double stepsize) {
    m_stepsize = stepsize;
    m_vehicle->SetStepsize(stepsize);
}

void ChCosimVehicleNode::Initialize(const ChCoordsys<>& chassisPos) {
    m_vehicle->Initialize(chassisPos);
    ////m_powertrain->Initialize();
    ////m_driver->Initialize();

    // Send wheel masses and inertias to the tire nodes
    double props[4];
    for (int iw = 0; iw < m_num_wheels; iw++) {
        double mass = m_vehicle->GetWheelBody(WheelID(iw))->GetMass();
        ChVector<> inertia = m_vehicle->GetWheelBody(WheelID(iw))->GetInertiaXX();
        props[0] = mass;
        props[1] = inertia.x;
        props[2] = inertia.y;
        props[3] = inertia.z;
        MPI_Send(props, 4, MPI_DOUBLE, TIRE_NODE(iw), iw, MPI_COMM_WORLD);
#ifdef VERBOSE_DEBUG
        printf("Vehicle node %d.  Send to %d  props = %g %g %g %g\n", m_rank, TIRE_NODE(iw), props[0], props[1],
               props[2], props[3]);
#endif
    }
}

void ChCosimVehicleNode::Synchronize(double time) {
    // Get current driver outputs
    double steering = m_driver->GetSteering();
    double throttle = m_driver->GetThrottle();
    double braking = m_driver->GetBraking();

    // Get current wheel states
    for (int iw = 0; iw < m_num_wheels; iw++) {
        m_wheel_states[iw] = m_vehicle->GetWheelState(WheelID(iw));
    }

    // Get current driveshaft speed and powertrain output torque
    double driveshaft_speed = m_vehicle->GetDriveshaftSpeed();
    double powertrain_torque = m_powertrain->GetOutputTorque();

    // Receive tire forces from each of the tire nodes
    //// TODO

    // Send wheel states to each of the tire nodes
    //// TODO

    // Synchronize vehicle, powertrain, and driver
    m_vehicle->Synchronize(time, steering, braking, powertrain_torque, m_tire_forces);
    m_powertrain->Synchronize(time, throttle, driveshaft_speed);
    m_driver->Synchronize(time);
}

void ChCosimVehicleNode::Advance(double step) {
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_vehicle->GetSystem()->DoStepDynamics(h);
        t += h;
    }
    m_powertrain->Advance(step);
    m_driver->Advance(step);
}

}  // end namespace vehicle
}  // end namespace chrono
