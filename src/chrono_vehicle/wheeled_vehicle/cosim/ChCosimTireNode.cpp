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
// Cosimulation node responsible for simulating a tire system.
//
// =============================================================================

#include <algorithm>
#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimManager.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimTireNode.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"

namespace chrono {
namespace vehicle {

ChCosimTireNode::ChCosimTireNode(ChSystem* system, ChTire* tire, WheelID id) : m_system(system), m_tire(tire), m_id(id) {}

void ChCosimTireNode::Initialize() {
    // Ghost wheel body (driven kinematically through messages from vehicle node)
    m_wheel = std::shared_ptr<ChBody>(m_system->NewBody());

    // Receive mass and inertia for the wheel body from the vehicle node
    //// TODO
    double props[4];
    MPI_Status status;
    MPI_Recv(props, 4, MPI_DOUBLE, VEHICLE_NODE_RANK, m_id.id(), MPI_COMM_WORLD, &status);

#ifdef VERBOSE_DEBUG
    printf("Tire node %d. Recv props = %g %g %g %g\n", m_rank, props[0], props[1], props[2], props[3]);
#endif

    m_wheel->SetMass(props[0]);
    m_wheel->SetInertiaXX(ChVector<>(props[1], props[2], props[3]));

    // Dummy terrain (needed for tire synchronization)
    m_terrain = std::make_shared<FlatTerrain>(0);
    ////m_terrain->Initialize(m_system);

    // Initialize the underlying tire
    m_tire->Initialize(m_wheel, m_id.side());
}

void ChCosimTireNode::Synchronize(double time) {
    // Send tire force to the vehicle node
    //// TODO

    // Receive wheel state from the vehicle node
    //// TODO

    // Send tire location(s) to the terrain node
    //// TODO

    // Receive terrain force(s) from the terrain node
    //// TODO

    // Synchronize the ghost wheel and the tire
    m_wheel->SetPos(m_wheel_state.pos);
    m_wheel->SetRot(m_wheel_state.rot);
    m_wheel->SetPos_dt(m_wheel_state.lin_vel);
    m_wheel->SetWvel_par(m_wheel_state.ang_vel);
    m_tire->Synchronize(time, m_wheel_state, *m_terrain);
}

void ChCosimTireNode::Advance(double step) {
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_tire->Advance(step);
}

}  // end namespace vehicle
}  // end namespace chrono
