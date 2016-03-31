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

ChCosimTerrainNode::ChCosimTerrainNode(int rank, ChSystem* system, ChTerrain* terrain, int num_tires)
    : ChCosimNode(rank, system), m_terrain(terrain), m_num_tires(num_tires) {}

void ChCosimTerrainNode::Initialize() {
    // Receive contact specification from tire nodes
    for (int it = 0; it < m_num_tires; it++) {
        unsigned int props[2];
        MPI_Status status;
        MPI_Recv(props, 2, MPI_UNSIGNED, TIRE_NODE_RANK(it), it, MPI_COMM_WORLD, &status);
        m_num_vertices.push_back(props[0]);
        m_num_triangles.push_back(props[1]);
        if (m_verbose) {
            printf("Terrain node %d.  Recv from %d props = %d %d\n", m_rank, TIRE_NODE_RANK(it), props[0], props[1]);
        }
    }
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
