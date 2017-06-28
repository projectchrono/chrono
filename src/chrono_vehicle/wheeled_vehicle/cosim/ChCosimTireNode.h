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
// Cosimulation node responsible for simulating a tire system.
//
// =============================================================================

#ifndef CH_COSIM_TIRE_NODE_H
#define CH_COSIM_TIRE_NODE_H

#include "mpi.h"

#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChLoadContactSurfaceMesh.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/cosim/ChCosimNode.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChCosimTireNode : public ChCosimNode {
  public:
    ChCosimTireNode(int rank, ChSystem* system, ChDeformableTire* tire, WheelID id);

    void Initialize();
    void Synchronize(double time);
    void Advance(double step);

  private:
    ChDeformableTire* m_tire;
    WheelID m_id;
    std::shared_ptr<ChBody> m_wheel;
    std::shared_ptr<ChTerrain> m_terrain;

    std::shared_ptr<fea::ChLoadContactSurfaceMesh> m_contact_load;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
