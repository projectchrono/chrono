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
// Generic rigid tire subsystem
//
// =============================================================================

#ifndef GENERIC_RIGID_TIRE_H
#define GENERIC_RIGID_TIRE_H

#include "subsys/tire/ChRigidTire.h"

class Generic_RigidTire : public chrono::ChRigidTire
{
public:

  Generic_RigidTire(const std::string&       name,
                    const chrono::ChTerrain& terrain)
  : ChRigidTire(name, terrain) {}

  ~Generic_RigidTire() {}

  virtual float getFrictionCoefficient() const { return 0.7f; }
  virtual double getRadius() const             { return 0.47; }
  virtual double getWidth() const              { return 0.25; }
};


#endif
