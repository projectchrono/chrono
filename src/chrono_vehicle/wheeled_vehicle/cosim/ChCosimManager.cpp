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
#include <cstdio>

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

bool ChCosimManager::Initialize() {
    // Initialize MPI
    int num_procs;
    MPI_Init(NULL, NULL);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &m_rank);

    if (num_procs != m_num_tires + 2) {
        if (m_rank == VEHICLE_NODE_RANK) {
            std::cout << "ERROR:  Incorrect number of processors!" << std::endl;
            std::cout << "  Provided: " << num_procs << std::endl;
            std::cout << "  Required: " << m_num_tires + 2 << std::endl;
        }
        return false;
    }

    // Create and initialize the different cosimulation nodes
    if (m_rank == VEHICLE_NODE_RANK) {
        SetNodeType(VEHICLE_NODE);
        m_vehicle_node = new ChCosimVehicleNode(m_rank, GetVehicle(), GetPowertrain(), GetDriver());
        if (m_num_tires != 2 * m_vehicle_node->GetNumberAxles()) {
            std::cout << "ERROR:  Incorrect number of tires!" << std::endl;
            std::cout << "  Provided: " << m_num_tires << std::endl;
            std::cout << "  Required: " << 2 * m_vehicle_node->GetNumberAxles() << std::endl;
            return false;
        }
        m_vehicle_node->Initialize(GetVehicleInitialPosition());
        m_vehicle_node->SetStepsize(GetVehicleStepsize());
#ifdef VERBOSE_DEBUG
        std::cout << "VEHICLE NODE created.  rank = " << m_rank << std::endl;
#endif
    }
    else if (m_rank == TERRAIN_NODE_RANK) {
        SetNodeType(TERRAIN_NODE);
        m_terrain_node = new ChCosimTerrainNode(m_rank, GetChronoSystemTerrain(), GetTerrain());
        m_terrain_node->Initialize();
        m_terrain_node->SetStepsize(GetTerrainStepsize());
#ifdef VERBOSE_DEBUG
        std::cout << "TERRAIN NODE created.  rank = " << m_rank << std::endl;
#endif
    }
    else {
        SetNodeType(TIRE_NODE);
        WheelID id(m_rank - 2);
        m_tire_node = new ChCosimTireNode(m_rank, GetChronoSystemTire(id), GetTire(id), id);
        m_tire_node->Initialize();
        m_tire_node->SetStepsize(GetTireStepsize(id));
#ifdef VERBOSE_DEBUG
        std::cout << "TIRE NODE created.  rank = " << m_rank << std::endl;
#endif
    }

    return true;
}

void ChCosimManager::Synchronize(double time) {
    if (m_rank == VEHICLE_NODE_RANK) {
        m_vehicle_node->Synchronize(time);
    } else if (m_rank == TERRAIN_NODE_RANK) {
        m_terrain_node->Synchronize(time);
    } else {
        m_tire_node->Synchronize(time);
    }
}

void ChCosimManager::Advance(double step) {
    if (m_rank == VEHICLE_NODE_RANK) {
        m_vehicle_node->Advance(step);
        OnAdvanceVehicle();
    } else if (m_rank == TERRAIN_NODE_RANK) {
        m_terrain_node->Advance(step);
        OnAdvanceTerrain();
    } else {
        WheelID id(m_rank - 2);
        m_tire_node->Advance(step);
        OnAdvanceTire(id);
    }
}

void ChCosimManager::Abort() {
    MPI_Abort(MPI_COMM_WORLD, 1);
}

}  // end namespace vehicle
}  // end namespace chrono
