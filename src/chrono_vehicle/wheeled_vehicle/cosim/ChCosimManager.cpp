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
// Manager for the distributed wheeled vehicle cosimulation.
//
// =============================================================================

#include <iostream>

#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimManager.h"

namespace chrono {
namespace vehicle {

ChCosimManager::ChCosimManager(int num_tires)
    : m_num_tires(num_tires), m_vehicle_node(NULL), m_terrain_node(NULL), m_tire_node(NULL) {}

ChCosimManager::~ChCosimManager() {
    delete m_vehicle_node;
    delete m_terrain_node;
    delete m_tire_node;

    MPI_Finalize();
}

bool ChCosimManager::Initialize(int argc, char** argv) {
    // Initialize MPI
    int num_procs;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &m_rank);

    if (num_procs != m_num_tires + 2) {
        if (m_rank == VEHICLE_NODE) {
            std::cout << "ERROR:  Incorrect number of processors!" << std::endl;
            std::cout << "  Provided:  " << num_procs << std::endl;
            std::cout << "  Required : " << m_num_tires + 2 << std::endl;
        }
        return false;
    }

    // Create and initialize the different cosimulation nodes
    if (m_rank == VEHICLE_NODE) {
        m_vehicle_node = new ChCosimVehicleNode(GetVehicle(), GetPowertrain(), GetDriver());
        if (m_num_tires != 2 * m_vehicle_node->GetNumberAxles()) {
            std::cout << "ERROR:  Incorrect number of tires!" << std::endl;
            std::cout << "  Provided:  " << m_num_tires << std::endl;
            std::cout << "  Required : " << 2 * m_vehicle_node->GetNumberAxles() << std::endl;
            return false;
        }
        m_vehicle_node->Initialize(GetVehicleInitialPosition());
        m_vehicle_node->SetStepsize(GetVehicleStepsize());
        m_vehicle_node->SetRank(m_rank);
    } else if (m_rank == TERRAIN_NODE) {
        m_terrain_node = new ChCosimTerrainNode(GetChronoSystemTerrain(), GetTerrain());
        m_terrain_node->Initialize();
        m_terrain_node->SetStepsize(GetTerrainStepsize());
        m_terrain_node->SetRank(m_rank);
    } else {
        m_tire_node = new ChCosimTireNode(GetChronoSystemTire(), GetTire(WheelID(m_rank - 2)), WheelID(m_rank - 2));
        m_tire_node->Initialize();
        m_tire_node->SetStepsize(GetTireStepsize());
        m_tire_node->SetRank(m_rank);
    }

    return true;
}

void ChCosimManager::Synchronize(double time) {
    if (m_rank == VEHICLE_NODE) {
        m_vehicle_node->Synchronize(time);
    } else if (m_rank == TERRAIN_NODE) {
        m_terrain_node->Synchronize(time);
    } else {
        m_tire_node->Synchronize(time);
    }
}

void ChCosimManager::Advance(double step) {

}

void ChCosimManager::Abort() {
    MPI_Abort(MPI_COMM_WORLD, 1);
}

}  // end namespace vehicle
}  // end namespace chrono
