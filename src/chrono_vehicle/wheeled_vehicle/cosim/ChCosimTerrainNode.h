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

#ifndef CH_COSIM_TERRAIN_NODE_H
#define CH_COSIM_TERRAIN_NODE_H

#include "mpi.h"

#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_cosim
/// @{

class CH_VEHICLE_API ChCosimTerrainNode {
public:
    ChCosimTerrainNode(ChSystem* system, ChTerrain* terrain);

    void SetRank(int rank) { m_rank = rank; }
    void SetStepsize(double stepsize) { m_stepsize = stepsize; }
    void Initialize();
    void Synchronize(double time);
    void Advance(double step);

private:
    int m_rank;
    
    ChSystem* m_system;
    ChTerrain* m_terrain;

    double m_stepsize;
};

/// @} vehicle_wheeled_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
