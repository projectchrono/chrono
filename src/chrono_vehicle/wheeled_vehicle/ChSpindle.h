// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Definition of a wheeled vehicle spindle.
// Each suspension subsystem creates and maintains two spindles, one for each
// side. A spindle object is a ChBody with an associated force/torque accumulator
// for applying tire/terrain forces.
//
// =============================================================================

#ifndef CH_SPINDLE_H
#define CH_SPINDLE_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadsBody.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

// Forward declaration
class ChWheel;

/// @addtogroup vehicle_wheeled_wheel
/// @{

/// Definition of a wheeled vehicle spindle.
/// Each suspension subsystem creates and maintains two spindles, one for each side. A spindle object is a ChBody with
/// an associated force/torque accumulator for applying tire/terrain forces.
class CH_VEHICLE_API ChSpindle : public ChBody {
  public:
    ChSpindle();
    ~ChSpindle() {}

    void AddTireAccumulator();
    void EmptyTireAccumulator();
    void AccumulateTireForce(const TerrainForce& tire_force);

    unsigned int GetTireAccumulatorIndex() const { return m_tire_accumulator_index; }

  private:
    unsigned int m_tire_accumulator_index;  ///< index of the accumulator for terrain/tire forces

    /*
    std::shared_ptr<ChLoadBodyForce> m_tire_force_load;    ///< terrain/tire force loads on the spindle
    std::shared_ptr<ChLoadBodyTorque> m_tire_torque_load;  ///< terrain/tire torque loads on the spindle
    */

    friend class ChWheel;
};

/// @} vehicle_wheeled_wheel

}  // end namespace vehicle
}  // end namespace chrono

#endif
