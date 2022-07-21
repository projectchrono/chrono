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
// Generic rigid tire subsystem
//
// =============================================================================

#ifndef ACV_RIGID_TIRE_H
#define ACV_RIGID_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

class ACV_RigidTire : public chrono::vehicle::ChRigidTire {
  public:
    ACV_RigidTire(const std::string& name);

    ~ACV_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetTireMass() const override { return m_mass; }
    virtual chrono::ChVector<> GetTireInertia() const override { return m_inertia; }

  private:
    virtual void CreateContactMaterial(chrono::ChContactMethod contact_method) override;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const chrono::ChVector<> m_inertia;
};

#endif
