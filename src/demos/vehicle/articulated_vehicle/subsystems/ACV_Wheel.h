// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
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

#ifndef ACV_WHEEL_H
#define ACV_WHEEL_H

#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

class ACV_Wheel : public chrono::vehicle::ChWheel {
  public:
    ACV_Wheel(const std::string& name);
    ~ACV_Wheel() {}

    virtual double GetWheelMass() const override { return m_mass; }
    virtual const chrono::ChVector<>& GetWheelInertia() const override { return m_inertia; }

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

  private:
    static const double m_mass;
    static const chrono::ChVector<> m_inertia;
    static const double m_radius;
    static const double m_width;
};

#endif
