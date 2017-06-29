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
// Manager for the distributed wheeled vehicle cosimulation.
//
// =============================================================================

#ifndef CH_COSIM_MANAGER_H
#define CH_COSIM_MANAGER_H

#include <vector>
#include "mpi.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimVehicleNode.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimTireNode.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimTerrainNode.h"

namespace chrono {
namespace vehicle {

#define VEHICLE_NODE_RANK 0
#define TERRAIN_NODE_RANK 1
#define TIRE_NODE_RANK(i) (i+2)

class CH_VEHICLE_API ChCosimManager {
  public:
    ChCosimManager(int num_tires);
    virtual ~ChCosimManager();

    // Functions invoked only on a VEHICLE node

    virtual void SetAsVehicleNode() {}
    virtual ChWheeledVehicle* GetVehicle() = 0;
    virtual ChPowertrain* GetPowertrain() = 0;
    virtual ChDriver* GetDriver() = 0;
    virtual double GetVehicleStepsize() = 0;
    virtual const ChCoordsys<>& GetVehicleInitialPosition() = 0;
    virtual void OnAdvanceVehicle() {}

    // Functions invoked only on a TERRAIN node

    virtual void SetAsTerrainNode() {}
    virtual ChSystem* GetChronoSystemTerrain() = 0;
    virtual ChTerrain* GetTerrain() = 0;
    virtual double GetTerrainStepsize() = 0;
    virtual void OnReceiveTireInfo(int which, unsigned int num_vert, unsigned int num_tri) = 0;
    virtual void OnReceiveTireData(int which,
                                   const std::vector<ChVector<>>& vert_pos,
                                   const std::vector<ChVector<>>& vert_vel,
                                   const std::vector<ChVector<int>>& triangles) = 0;
    virtual void OnSendTireForces(int which, std::vector<ChVector<>>& vert_forces, std::vector<int> vert_indeces) = 0;
    virtual void OnAdvanceTerrain() {}

    // Functions invoked only on a TIRE node

    virtual void SetAsTireNode(WheelID which) {}
    virtual ChSystem* GetChronoSystemTire(WheelID which) = 0;
    virtual ChDeformableTire* GetTire(WheelID which) = 0;
    virtual double GetTireStepsize(WheelID which) = 0;
    virtual void OnAdvanceTire(WheelID id) {}

    void SetVerbose(bool val) { m_verbose = val; }

    bool Initialize();
    void Abort();

    void Synchronize(double time);
    void Advance(double step);

  private:
    int m_rank;
    int m_num_tires;
    bool m_verbose;

    ChCosimVehicleNode* m_vehicle_node;
    ChCosimTerrainNode* m_terrain_node;
    ChCosimTireNode* m_tire_node;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
