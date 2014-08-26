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

#ifndef HMMWV_RIGIDTIRE_H
#define HMMWV_RIGIDTIRE_H


#include "subsys/tire/ChRigidTire.h"

namespace hmmwv {

class HMMWV9_RigidTire : public chrono::ChRigidTire {
public:
  HMMWV9_RigidTire(const chrono::ChTerrain& terrain,
                   float                    mu);
  ~HMMWV9_RigidTire() {}

  virtual float getFrictionCoefficient() const { return m_mu; }
  virtual double getRadius() const             { return m_radius; }
  virtual double getWidth() const              { return m_width; }

private:
  float                m_mu;

  static const double  m_radius;
  static const double  m_width;
};


} // end namespace hmmwv


#endif
