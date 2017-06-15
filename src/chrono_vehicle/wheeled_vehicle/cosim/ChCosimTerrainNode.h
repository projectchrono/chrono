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

#include <vector>
#include "mpi.h"

#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimNode.h"

namespace chrono {
namespace vehicle {

class ChCosimManager;

class CH_VEHICLE_API ChCosimTerrainNode : public ChCosimNode {
  public:
    ChCosimTerrainNode(int rank, ChSystem* system, ChTerrain* terrain, int num_tires);

    void Initialize();
    void Synchronize(double time);
    void Advance(double step);

  private:
    ChCosimManager* m_manager;                  // back-pointer to the cosimulation manager
    ChTerrain* m_terrain;                       // underlying terrain object
    int m_num_tires;                            // number of tires
    std::vector<unsigned int> m_num_vertices;   // number of contact vertices received from each tire
    std::vector<unsigned int> m_num_triangles;  // number of contact triangles received from each tire

    friend class ChCosimManager;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
