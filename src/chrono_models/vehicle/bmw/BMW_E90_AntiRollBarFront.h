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
// Authors: Rainer Gericke
// =============================================================================
//
// BMW E90 antirollbar RSD model.
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// =============================================================================

#ifndef BMW_E90_ANTIROLLBAR_FRONT_H
#define BMW_E90_ANTIROLLBAR_FRONT_H

#include "chrono_vehicle/wheeled_vehicle/antirollbar/ChAntirollBarRSD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Anti-roll bar subsystem for a FEDA vehicle.
class CH_MODELS_API BMW_E90_AntiRollBarFront : public ChAntirollBarRSD {
  public:
    BMW_E90_AntiRollBarFront(const std::string& name);
    ~BMW_E90_AntiRollBarFront() {}

    virtual double getArmMass() const override { return m_arm_mass; }
    virtual ChVector3d getArmInertia() override { return m_arm_inertia; }

    virtual double getArmLength() const override { return m_arm_length; }
    virtual double getArmWidth() const override { return m_arm_width; }
    virtual double getDroplinkHeight() const override { return m_droplink_height; }
    virtual double getArmRadius() const override { return m_arm_radius; }

    virtual double getSpringCoefficient() const override { return m_spring_coef; }
    virtual double getDampingCoefficient() const override { return m_damping_coef; }

  private:
    static const double m_arm_mass;
    static const ChVector3d m_arm_inertia;
    static const double m_arm_length;
    static const double m_arm_width;
    static const double m_droplink_height;
    static const double m_arm_radius;
    static const double m_spring_coef;
    static const double m_damping_coef;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
