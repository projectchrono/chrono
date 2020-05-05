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
// LMTV rotary arm steering model.
//
// =============================================================================

#ifndef LMTV_ROTARY_ARM_H
#define LMTV_ROTARY_ARM_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mtv {

/// @addtogroup vehicle_models_mtv
/// @{

/// RotaryArm steering subsystem for the LMTV vehicle.
class CH_MODELS_API LMTV_RotaryArm : public ChRotaryArm {
  public:
    LMTV_RotaryArm(const std::string& name);
    ~LMTV_RotaryArm() {}

    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector<>& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector<>& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const ChVector<> getLocation(PointId which) override;
    virtual const ChVector<> getDirection(DirectionId which) override;

  private:
    static const double m_pitmanArmMass;

    static const double m_pitmanArmRadius;

    static const double m_maxAngle;

    static const ChVector<> m_pitmanArmInertiaMoments;
    static const ChVector<> m_pitmanArmInertiaProducts;
};

/// @} vehicle_models_mtv

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
