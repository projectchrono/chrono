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
// Authors: Justin Madsen
// =============================================================================
//
// Simple flat horizontal terrain (infinite x-y extent)
//
// =============================================================================


#ifndef FLATTERRAIN_H
#define FLATTERRAIN_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChTerrain.h"

namespace chrono {

///
/// Concrete class for a flat horizontal terrain.
/// This class implements a terrain modeled as an infinite horizontal plane at
/// a specified height.  This type of terrain can be used in conjunction with
/// tire models that perform their own collision detection (e.g. ChPacejkaTire
/// and ChLugreTire).
///
class CH_SUBSYS_API FlatTerrain : public ChTerrain
{
public:

  FlatTerrain(
    const int height   ///< [in] terrain height
    );

  ~FlatTerrain() {}

  /// Get the terrain height at the specified (x,y) location.
  /// Returns the constant value passed at construction.
  virtual double GetHeight(double x, double y) const { return m_height; }

  /// Get the terrain normal at the specified (x,y) location.
  /// Returns a constant unit vector along the Z axis.
  virtual ChVector<> GetNormal(double x, double y) const { return ChVector<>(0, 0, 1); }

private:

  double m_height;
};


} // end namespace chrono


#endif
