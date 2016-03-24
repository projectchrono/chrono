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
// Cosimulation node responsible for simulating a terrain system.
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimManager.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimTerrainNode.h"

namespace chrono {
namespace vehicle {

ChCosimTerrainNode::ChCosimTerrainNode(ChSystem* system, ChTerrain* terrain) : m_system(system), m_terrain(terrain) {}

void ChCosimTerrainNode::Initialize() {
    ////m_terrain->Initialize(m_system);
}

void ChCosimTerrainNode::Synchronize(double time) {
    // Receive tire location(s) from each of the tire nodes
    //// TODO

    // Send terrain force(s) to each of the tire nodes
    //// TODO

    m_terrain->Synchronize(time);
}

void ChCosimTerrainNode::Advance(double step) {
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_terrain->Advance(step);
}

}  // end namespace vehicle
}  // end namespace chrono
