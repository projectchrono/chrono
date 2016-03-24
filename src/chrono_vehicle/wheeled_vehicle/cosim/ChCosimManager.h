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

#ifndef CH_COSIM_MANAGER_H
#define CH_COSIM_MANAGER_H

#include "mpi.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimVehicleNode.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimTireNode.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimTerrainNode.h"

/**
    @addtogroup vehicle_wheeled
    @{
        @defgroup vehicle_wheeled_cosim Cosimulation infrastructure
    @}
*/

namespace chrono {
namespace vehicle {

#define VERBOSE_DEBUG

#define VEHICLE_NODE 0
#define TERRAIN_NODE 1
#define TIRE_NODE(i) (i+2)

/// @addtogroup vehicle_wheeled_cosim
/// @{

class CH_VEHICLE_API ChCosimManager {
  public:
    ChCosimManager(int num_tires);
    virtual ~ChCosimManager();

    virtual ChWheeledVehicle* GetVehicle() = 0;
    virtual ChPowertrain* GetPowertrain() = 0;
    virtual ChDriver* GetDriver() = 0;
    virtual double GetVehicleStepsize() = 0;

    virtual ChSystem* GetChronoSystemTerrain() = 0;
    virtual ChTerrain* GetTerrain() = 0;
    virtual double GetTerrainStepsize() = 0;

    virtual ChSystem* GetChronoSystemTire() = 0;
    virtual ChTire* GetTire(WheelID which) = 0;
    virtual double GetTireStepsize() = 0;

    virtual const ChCoordsys<>& GetVehicleInitialPosition() = 0;

    bool Initialize(int argc, char** argv);
    void Abort();

    void Synchronize(double time);
    void Advance(double step);

    virtual void OnAdvanceVehicle() {}
    virtual void OnAdvanceTerrain() {}
    virtual void OnAdvanceTire(WheelID id) {}

  private:
    int m_rank;
    int m_num_tires;

    ChCosimVehicleNode* m_vehicle_node;
    ChCosimTerrainNode* m_terrain_node;
    ChCosimTireNode* m_tire_node;
};

/// @} vehicle_wheeled_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
