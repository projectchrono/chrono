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
// HMMWV Pitman arm steering model with compliant steering column.
//
// =============================================================================

#ifndef HMMWV_PITMAN_ARM_SHAFTS_H
#define HMMWV_PITMAN_ARM_SHAFTS_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArmShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Pitman-arm with compliant column steering subsystem for the HMMWV vehicle.
class CH_MODELS_API HMMWV_PitmanArmShafts : public ChPitmanArmShafts {
  public:
    HMMWV_PitmanArmShafts(const std::string& name, bool rigid_connection);
    ~HMMWV_PitmanArmShafts() {}

    virtual double getSteeringLinkMass() const override { return m_steeringLinkMass; }
    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getSteeringLinkRadius() const override { return m_steeringLinkRadius; }
    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector<>& getSteeringLinkInertiaMoments() const override { return m_steeringLinkInertiaMoments; }
    virtual const ChVector<>& getSteeringLinkInertiaProducts() const override { return m_steeringLinkInertiaProducts; }
    virtual const ChVector<>& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector<>& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual double getMaxAngle() const override { return m_maxAngle; }
    virtual double getGearRatio() const override { return m_gearRatio; }
    virtual double getSteeringCompliance() const override { return m_steeringCompliance; }
    virtual double getSteeringColumnInertia() const override { return m_columnInertia; }

    virtual const ChVector<> getLocation(PointId which) override;
    virtual const ChVector<> getDirection(DirectionId which) override;

  private:
    static const double m_steeringLinkMass;
    static const double m_pitmanArmMass;

    static const double m_steeringLinkRadius;
    static const double m_pitmanArmRadius;

    static const double m_maxAngle;
    static const double m_gearRatio;
    static const double m_steeringCompliance;
    static const double m_columnInertia;

    static const ChVector<> m_steeringLinkInertiaMoments;
    static const ChVector<> m_steeringLinkInertiaProducts;
    static const ChVector<> m_pitmanArmInertiaMoments;
    static const ChVector<> m_pitmanArmInertiaProducts;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
