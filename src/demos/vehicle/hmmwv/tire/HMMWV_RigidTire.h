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

#ifndef HMMWV_RIGID_TIRE_H
#define HMMWV_RIGID_TIRE_H

#include "chrono_vehicle/tire/ChRigidTire.h"

namespace hmmwv {

class HMMWV_RigidTire : public chrono::ChRigidTire {
  public:
    HMMWV_RigidTire(const std::string& name);
    ~HMMWV_RigidTire() {}

    virtual double getRadius() const override { return m_radius; }
    virtual double getWidth() const override { return m_width; }

  private:
    static const double m_radius;
    static const double m_width;
};

}  // end namespace hmmwv

#endif
