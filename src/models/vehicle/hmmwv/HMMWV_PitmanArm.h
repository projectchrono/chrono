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
// HMMWV Pitman arm steering model.
//
// =============================================================================

#ifndef HMMWV_PITMAN_ARM_H
#define HMMWV_PITMAN_ARM_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArm.h"

#include "models/ChApiModels.h"

namespace hmmwv {

class CH_MODELS_API HMMWV_PitmanArm : public chrono::vehicle::ChPitmanArm {
  public:
    HMMWV_PitmanArm(const std::string& name);
    ~HMMWV_PitmanArm() {}

    virtual double getSteeringLinkMass() const override { return m_steeringLinkMass; }
    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getSteeringLinkRadius() const override { return m_steeringLinkRadius; }
    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const chrono::ChVector<>& getSteeringLinkInertia() const override { return m_steeringLinkInertia; }
    virtual const chrono::ChVector<>& getPitmanArmInertia() const override { return m_pitmanArmInertia; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const chrono::ChVector<> getLocation(PointId which) override;
    virtual const chrono::ChVector<> getDirection(DirectionId which) override;

  private:
    static const double m_steeringLinkMass;
    static const double m_pitmanArmMass;

    static const double m_steeringLinkRadius;
    static const double m_pitmanArmRadius;

    static const double m_maxAngle;

    static const chrono::ChVector<> m_steeringLinkInertia;
    static const chrono::ChVector<> m_pitmanArmInertia;
};

}  // end namespace hmmwv

#endif
