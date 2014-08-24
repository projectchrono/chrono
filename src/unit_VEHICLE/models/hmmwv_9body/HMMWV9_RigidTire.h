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
// HMMWV rigid tire subsystem
//
// =============================================================================

#ifndef HMMWV9_RIGIDTIRE_H
#define HMMWV9_RIGIDTIRE_H

#include "physics/ChBody.h"

#include "subsys/ChTire.h"

#include "HMMWV9.h"

namespace hmmwv9 {

class HMMWV9_RigidTire : public chrono::ChTire {
public:

  HMMWV9_RigidTire(const chrono::ChTerrain& terrain,
                   float                    mu);

  ~HMMWV9_RigidTire() {}

  virtual chrono::ChTireForce GetTireForce() const;

  void Initialize(chrono::ChSharedBodyPtr wheel);

private:

  float                m_mu;

  static const double  m_radius;
  static const double  m_width;
};


} // end namespace hmmwv9


#endif
