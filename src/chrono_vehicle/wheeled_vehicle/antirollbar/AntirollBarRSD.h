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
// RSD antirollbar model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef ANTIROLLBAR_RSD_H
#define ANTIROLLBAR_RSD_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/antirollbar/ChAntirollBarRSD.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_antirollbar
/// @{

/// RSD antirollbar model constructed with data from file (JSON format).
class CH_VEHICLE_API AntirollBarRSD : public ChAntirollBarRSD {
  public:
    AntirollBarRSD(const std::string& filename);
    AntirollBarRSD(const rapidjson::Document& d);
    ~AntirollBarRSD() {}

    virtual double getArmMass() const override { return m_arm_mass; }
    virtual ChVector<> getArmInertia() override { return m_arm_inertia; }

    virtual double getArmLength() const override { return m_arm_length; }
    virtual double getArmWidth() const override { return m_arm_width; }
    virtual double getDroplinkHeight() const override { return m_link_height; }
    virtual double getArmRadius() const override { return m_arm_radius; }

    virtual double getSpringCoefficient() const override { return m_spring_coef; }
    virtual double getDampingCoefficient() const override { return m_damping_coef; }

  private:
    void Create(const rapidjson::Document& d);

    double m_arm_mass;
    ChVector<> m_arm_inertia;

    double m_arm_length;
    double m_arm_width;
    double m_link_height;
    double m_arm_radius;

    double m_spring_coef;
    double m_damping_coef;
};

/// @} vehicle_wheeled_antirollbar

}  // end namespace vehicle
}  // end namespace chrono

#endif
