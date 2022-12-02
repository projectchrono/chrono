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
// Authors: Justin Madsen
// =============================================================================
//
// Simple flat horizontal terrain (infinite x-y extent)
//
// =============================================================================

#ifndef FLATTERRAIN_H
#define FLATTERRAIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Concrete class for a flat horizontal terrain.
/// This class implements a terrain modeled as an infinite horizontal plane at a specified height.
/// This type of terrain can be used in conjunction with tire models that perform their own collision detection
/// (e.g., ChTMeasy ChPac89, ChPac02, ChFiala).
class CH_VEHICLE_API FlatTerrain : public ChTerrain {
  public:
    FlatTerrain(double height,         ///< [in] terrain height
                float friction = 0.8f  ///< [in] terrain coefficient of friction
                );

    ~FlatTerrain() {}

    /// Get the terrain height below the specified location.
    /// Returns the constant value passed at construction.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    /// Returns a constant unit vector along the vertical axis.
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For FlatTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified at construction.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

  private:
    double m_height;   ///< terrain height
    float m_friction;  ///< contact coefficient of friction
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
