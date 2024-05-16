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
// Authors: Alessandro Tasora, Radu Serban, Asher Elmquist, Rainer Gericke
// data from http://www.treffseiten.de/bmw/info/daten_320i_325i_330i_320d_limousine_05_09.pdf
// Conical gear ratio from automatic gearbox version
// =============================================================================
//
// BMW E90 (330i 2006) 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef BMW_E90_DRIVELINE_H
#define BMW_E90_DRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Shafts-based 2-WD driveline for the Sedan vehicle.
class CH_MODELS_API BMW_E90_Driveline : public ChShaftsDriveline2WD {
  public:
    BMW_E90_Driveline(const std::string& name);

    ~BMW_E90_Driveline() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetDifferentialBoxInertia() const override { return m_differentialbox_inertia; }

    virtual double GetConicalGearRatio() const override { return m_conicalgear_ratio; }

    virtual double GetAxleDifferentialLockingLimit() const override { return m_axle_differential_locking_limit; }

  private:
    // Shaft inertias
    static const double m_driveshaft_inertia;
    static const double m_differentialbox_inertia;

    // Gear ratio
    static const double m_conicalgear_ratio;

    // Differential locking torque limit.
    static const double m_axle_differential_locking_limit;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
