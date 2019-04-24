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
// Base class for a terrain subsystem.
//
// =============================================================================

#ifndef CH_TERRAIN_H
#define CH_TERRAIN_H

#include "chrono/core/ChVector.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Base class for a terrain system.
class CH_VEHICLE_API ChTerrain {
  public:
    ChTerrain();

    virtual ~ChTerrain() {}

    /// Update the state of the terrain system at the specified time.
    virtual void Synchronize(double time) {}

    /// Advance the state of the terrain system by the specified duration.
    virtual void Advance(double step) {}

    /// Get the terrain height at the specified (x,y) location.
    virtual double GetHeight(double x, double y) const = 0;

    /// Get the terrain normal at the specified (x,y) location.
    virtual ChVector<> GetNormal(double x, double y) const = 0;

    /// Get the terrain coefficient of friction at the specified (x,y) location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    virtual float GetCoefficientFriction(double x, double y) const = 0;

    /// Class to be used as a functor interface for location-dependent coefficient of friction.
    class CH_VEHICLE_API FrictionFunctor {
      public:
        virtual ~FrictionFunctor() {}

        /// Return the coefficient of friction at a given (x,y) location.
        virtual float operator()(double x, double y) = 0;
    };

    /// Specify the functor object to provide the coefficient of friction at given (x,y) locations.
    void RegisterFrictionFunctor(FrictionFunctor* functor) { m_friction_fun = functor; }

  protected:
    FrictionFunctor* m_friction_fun;  ///< functor for location-dependent coefficient of friction
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
