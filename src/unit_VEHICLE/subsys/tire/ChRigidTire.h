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
// Generic rigid tire
//
// =============================================================================

#ifndef CH_RIGIDTIRE_H
#define CH_RIGIDTIRE_H

#include "physics/ChBody.h"

#include "subsys/ChTire.h"
#include "subsys/ChTerrain.h"

namespace chrono {

class CH_SUBSYS_API ChRigidTire : public ChTire {
public:
  ChRigidTire(const ChTerrain& terrain);
  virtual ~ChRigidTire() {}

  virtual ChTireForce GetTireForce() const;

  void Initialize(ChSharedBodyPtr wheel);

protected:
  virtual float getFrictionCoefficient() const = 0;
  virtual double getRadius() const = 0;
  virtual double getWidth() const = 0;
};


} // end namespace chrono


#endif
