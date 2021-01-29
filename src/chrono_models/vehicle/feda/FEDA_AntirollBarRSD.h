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
// FED alpha antirollbar RSD model.
//
// =============================================================================

#ifndef FEDA_ANTIROLLBAR_RSD_H
#define FEDA_ANTIROLLBAR_RSD_H

#include "chrono_vehicle/wheeled_vehicle/antirollbar/ChAntirollBarRSD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace feda {

/// @addtogroup vehicle_models_feda
/// @{

/// Anti-roll bar subsystem for a FEDA vehicle.
class CH_MODELS_API FEDA_AntirollBarRSD : public ChAntirollBarRSD {
  public:
    FEDA_AntirollBarRSD(const std::string& name);
    ~FEDA_AntirollBarRSD() {}

    virtual double getArmMass() const override { return m_arm_mass; }
    virtual ChVector<> getArmInertia() override { return m_arm_inertia; }

    virtual double getArmLength() const override { return m_arm_length; }
    virtual double getArmWidth() const override { return m_arm_width; }
    virtual double getDroplinkHeight() const override { return m_droplink_height; }
    virtual double getArmRadius() const override { return m_arm_radius; }

    virtual double getSpringCoefficient() const override { return m_spring_coef; }
    virtual double getDampingCoefficient() const override { return m_damping_coef; }

  private:
    static const double m_arm_mass;
    static const ChVector<> m_arm_inertia;
    static const double m_arm_length;
    static const double m_arm_width;
    static const double m_droplink_height;
    static const double m_arm_radius;
    static const double m_spring_coef;
    static const double m_damping_coef;
};

/// @} vehicle_models_feda

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono

#endif
