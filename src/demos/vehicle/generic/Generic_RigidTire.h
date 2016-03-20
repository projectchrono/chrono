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

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

class Generic_RigidTire : public chrono::vehicle::ChRigidTire {
  public:
    Generic_RigidTire(const std::string& name) : chrono::vehicle::ChRigidTire(name) {
        SetContactMaterial(0.9f, 0.1f, 2e7f, 0.3f);
    }

    ~Generic_RigidTire() {}

    virtual double GetRadius() const override { return 0.47; }
    virtual double GetWidth() const override { return 0.25; }
};

#endif
