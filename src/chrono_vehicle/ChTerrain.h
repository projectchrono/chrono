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
// Base class for a terrain subsystem.
//
// =============================================================================

#ifndef CH_TERRAIN_H
#define CH_TERRAIN_H

#include "chrono/core/ChVector.h"

#include "chrono_vehicle/ChApiVehicle.h"

/**
    @addtogroup vehicle
    @{
        @defgroup vehicle_terrain Terrain models
    @}
*/

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Base class for a terrain system.
class CH_VEHICLE_API ChTerrain {
  public:
    ChTerrain() {}

    virtual ~ChTerrain() {}

    /// Update the state of the terrain system at the specified time.
    virtual void Synchronize(double time) {}

    /// Advance the state of the terrain system by the specified duration.
    virtual void Advance(double step) {}

    /// Get the terrain height at the specified (x,y) location.
    virtual double GetHeight(double x, double y) const = 0;

    /// Get the terrain normal at the specified (x,y) location.
    virtual ChVector<> GetNormal(double x, double y) const = 0;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
