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
// Generic wheel subsystem
//
// =============================================================================

#ifndef GENERIC_WHEEL_H
#define GENERIC_WHEEL_H

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

class Generic_Wheel : public chrono::vehicle::ChWheel {
  public:
    Generic_Wheel(const std::string& name) : ChWheel(name) {}
    ~Generic_Wheel() {}

    virtual double GetMass() const override { return 45.4; }
    virtual chrono::ChVector<> GetInertia() const override { return chrono::ChVector<>(0.113, 0.113, 0.113); }

    virtual double GetRadius() const override { return 0.268; }
    virtual double GetWidth() const override { return 0.22; }
};

#endif
