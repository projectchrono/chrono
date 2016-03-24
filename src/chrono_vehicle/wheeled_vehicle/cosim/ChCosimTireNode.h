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

#ifndef CH_COSIM_TIRE_NODE_H
#define CH_COSIM_TIRE_NODE_H

#include "mpi.h"

#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_cosim
/// @{

class CH_VEHICLE_API ChCosimTireNode {
  public:
    ChCosimTireNode(ChSystem* system, ChTire* tire, WheelID id);
    ~ChCosimTireNode() {}

    void SetRank(int rank) { m_rank = rank; }
    void SetStepsize(double stepsize) { m_stepsize = stepsize; }
    void Initialize();
    void Synchronize(double time);
    void Advance(double step);

  private:
    int m_rank;

    ChSystem* m_system;
    ChTire* m_tire;
    WheelID m_id;
    std::shared_ptr<ChBody> m_wheel;
    std::shared_ptr<ChTerrain> m_terrain;

    WheelState m_wheel_state;

    double m_stepsize;
};

/// @} vehicle_wheeled_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
