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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Kraz 64431 rotary arm steering model.
//
// =============================================================================

#ifndef SEMITRACTOR_ROTARY_ARM_H
#define SEMITRACTOR_ROTARY_ARM_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

#include "chrono_models/ChApiModels.h"

class SemiTractor_steering : public chrono::vehicle::ChRotaryArm {
  public:
    SemiTractor_steering(const std::string& name);
    ~SemiTractor_steering() {}

    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const chrono::ChVector<>& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const chrono::ChVector<>& getPitmanArmInertiaProducts() const override {
        return m_pitmanArmInertiaProducts;
    }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const chrono::ChVector<> getLocation(PointId which) override;
    virtual const chrono::ChVector<> getDirection(DirectionId which) override;

  private:
    static const double m_pitmanArmMass;

    static const double m_pitmanArmRadius;

    static const double m_maxAngle;

    static const chrono::ChVector<> m_pitmanArmInertiaMoments;
    static const chrono::ChVector<> m_pitmanArmInertiaProducts;
};

#endif
