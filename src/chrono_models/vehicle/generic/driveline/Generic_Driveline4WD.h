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
// Generic vehicle 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef GENERIC_DRIVELINE_4WD_H
#define GENERIC_DRIVELINE_4WD_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Shafts-based 4-WD driveline for the generic vehicle.
class CH_MODELS_API Generic_Driveline4WD : public ChShaftsDriveline4WD {
  public:
    Generic_Driveline4WD(const std::string& name);
    ~Generic_Driveline4WD() {}

    virtual double GetCentralDifferentialBoxInertia() const override { return m_central_differentialbox_inertia; }
    virtual double GetFrontDifferentialBoxInertia() const override { return m_front_differentialbox_inertia; }
    virtual double GetRearDifferentialBoxInertia() const override { return m_rear_differentialbox_inertia; }
    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetToFrontDiffShaftInertia() const override { return m_frontshaft_inertia; }
    virtual double GetToRearDiffShaftInertia() const override { return m_rearshaft_inertia; }

    virtual double GetFrontConicalGearRatio() const override { return m_front_conicalgear_ratio; }
    virtual double GetRearConicalGearRatio() const override { return m_rear_conicalgear_ratio; }

    virtual double GetAxleDifferentialLockingLimit() const override { return m_axle_differential_locking_limit; }
    virtual double GetCentralDifferentialLockingLimit() const override { return m_central_differential_locking_limit; }

  private:
    // Shaft inertias.
    static const double m_central_differentialbox_inertia;
    static const double m_front_differentialbox_inertia;
    static const double m_rear_differentialbox_inertia;
    static const double m_driveshaft_inertia;
    static const double m_frontshaft_inertia;
    static const double m_rearshaft_inertia;

    // Gear ratios.
    static const double m_front_conicalgear_ratio;
    static const double m_rear_conicalgear_ratio;

    // Differential locking torque limits.
    static const double m_axle_differential_locking_limit;
    static const double m_central_differential_locking_limit;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
