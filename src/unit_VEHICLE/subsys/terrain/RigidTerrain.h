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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple flat rigid terrain
//
// =============================================================================

#ifndef RIGIDTERRAIN_H
#define RIGIDTERRAIN_H

#include "physics/ChSystem.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChTerrain.h"


namespace chrono {

///
/// Concrete class for a rigid flat terrain.
/// This class implements a terrain modeled as a rigid box which can interact
/// through contact anf friction with any other bodies whose contact flag is
/// enabled. In particular, this type of terrain can be used in conjunction with
/// a ChRigidTire.
///
class CH_SUBSYS_API RigidTerrain : public ChTerrain
{
public:

  RigidTerrain(
    chrono::ChSystem&  system,   ///< [in] handle to the containing multibody system
    double             height,   ///< [in] terrain height
    double             sizeX,    ///< [in] terrain dimension in the X direction
    double             sizeY,    ///< [in] terrain dimension in the Y direction
    double             mu,        ///< [in] coefficient of friction
    const std::string  road_file = "none"
    );

  ~RigidTerrain() {}

  /// Get the terrain height at the specified (x,y) location.
  /// Returns the constant value passed at construction.
  virtual double GetHeight(double x, double y) const { return m_height; }

  /// Get the terrain normal at the specified (x,y) location.
  /// Returns a constant unit vector along the Z axis.
  virtual chrono::ChVector<> GetNormal(double x, double y) const { return chrono::ChVector<>(0, 0, 1); }

  /// Add the specified number of rigid bodies, modeled as boxes of random size
  /// and created at random locations above the terrain.
  void AddMovingObstacles(int numObstacles);

  /// Add a few contact objects, rigidly attached to the terrain.
  void AddFixedObstacles();

private:

  ChSystem&  m_system;
  double     m_sizeX;
  double     m_sizeY;
  double     m_height;
};


} // end namespace chrono


#endif
