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
// Simple rigid terrain...
//
// =============================================================================

#ifndef HMMWV_RIGIDTERRAIN_H
#define HMMWV_RIGIDTERRAIN_H

#include "physics/ChSystem.h"

#include "subsys/ChTerrain.h"


namespace hmmwv {

class HMMWV_RigidTerrain : public chrono::ChTerrain {
public:

  HMMWV_RigidTerrain(chrono::ChSystem&  system,
                     double             height,
                     double             sizeX,
                     double             sizeY,
                     double             mu);

  ~HMMWV_RigidTerrain() {}

  virtual double GetHeight(double x, double y) const { return m_height; }

  void AddMovingObstacles(int numObstacles);
  void AddFixedObstacles();

private:

  chrono::ChSystem&  m_system;

  double m_sizeX;
  double m_sizeY;
  double m_height;
};


} // end namespace hmmwv


#endif