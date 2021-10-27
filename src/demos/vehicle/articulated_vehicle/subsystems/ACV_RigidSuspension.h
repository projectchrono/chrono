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
// Generic rigid suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChRigidSuspension) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef ACV_RIGID_SUSPENSION_H
#define ACV_RIGID_SUSPENSION_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidSuspension.h"

class ACV_RigidSuspension : public chrono::vehicle::ChRigidSuspension {
  public:
    ACV_RigidSuspension(const std::string& name);
    ~ACV_RigidSuspension() {}

    virtual const chrono::ChVector<> getLocation(PointId which) override;

    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const chrono::ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

  private:
    static const double m_spindleMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const chrono::ChVector<> m_spindleInertia;

    static const double m_axleInertia;
};

#endif
