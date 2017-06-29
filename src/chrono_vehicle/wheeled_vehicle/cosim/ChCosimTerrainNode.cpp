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

        m_manager->OnReceiveTireInfo(it, props[0], props[1]);
    }
}

void ChCosimTerrainNode::Synchronize(double time) {
    for (int it = 0; it < m_num_tires; it++) {
        // Receive tire mesh vertex locations and velocities from the tire node
        MPI_Status status;
        unsigned int num_vert = m_num_vertices[it];
        unsigned int num_tri = m_num_triangles[it];
        double* vert_data = new double[2 * 3 * num_vert];
        int* tri_data = new int[3 * num_tri];
        MPI_Recv(vert_data, 2 * 3 * num_vert, MPI_DOUBLE, TIRE_NODE_RANK(it), it, MPI_COMM_WORLD, &status);
        MPI_Recv(tri_data, 3 * num_tri, MPI_INT, TIRE_NODE_RANK(it), it, MPI_COMM_WORLD, &status);

        // Unpack received data
        std::vector<ChVector<>> vert_pos;
        std::vector<ChVector<>> vert_vel;
        std::vector<ChVector<int>> triangles;
        for (unsigned int i = 0; i < num_vert; i++) {
            vert_pos.push_back(ChVector<>(vert_data[3 * i + 0], vert_data[3 * i + 1], vert_data[3 * i + 2]));
            vert_vel.push_back(ChVector<>(vert_data[3 * num_vert + 3 * i + 0], vert_data[3 * num_vert + 3 * i + 1],
                                          vert_data[3 * num_vert + 3 * i + 2]));
        }
        for (unsigned int i = 0; i < num_tri; i++) {
            triangles.push_back(ChVector<int>(tri_data[3 * i + 0], tri_data[3 * i + 1], tri_data[3 * i + 2]));
        }

        delete[] vert_data;
        delete[] tri_data;

        // Let derived class process received data
        m_manager->OnReceiveTireData(it, vert_pos, vert_vel, triangles);

        // Let derived class produce tire contact forces
        std::vector<ChVector<>> vert_forces;
        std::vector<int> vert_indeces;
        m_manager->OnSendTireForces(it, vert_forces, vert_indeces);
        num_vert = (unsigned int)vert_indeces.size();

        // Send vertex indeces and forces to the tire node
        //// TODO: use custom derived MPI types?
        double* force_data = new double[3 * num_vert];
        for (unsigned int i = 0; i < num_vert; i++) {
            force_data[3 * i + 0] = vert_forces[i].x;
            force_data[3 * i + 1] = vert_forces[i].y;
            force_data[3 * i + 2] = vert_forces[i].z;
        }
        MPI_Send(vert_indeces.data(), num_vert, MPI_INT, TIRE_NODE_RANK(it), it, MPI_COMM_WORLD);
        MPI_Send(force_data, 3 * num_vert, MPI_DOUBLE, TIRE_NODE_RANK(it), it, MPI_COMM_WORLD);

        delete[] force_data;
    }

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
