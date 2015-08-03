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

#include "core/ChShared.h"
#include "core/ChVector.h"

#include "subsys/ChApiSubsys.h"


namespace chrono {

///
/// Base class for a height-field terrain system.
///
class CH_SUBSYS_API ChTerrain : public ChShared
{
public:

  ChTerrain() {}
  virtual ~ChTerrain() {}

  virtual void Update(double time) {}
  virtual void Advance(double step) {}

  /// Get the terrain height at the specified (x,y) location.
  virtual double GetHeight(double x, double y) const = 0;

  /// Get the terrain normal at the specified (x,y) location.
  virtual ChVector<> GetNormal(double x, double y) const = 0;
};


} // end namespace chrono


#endif
