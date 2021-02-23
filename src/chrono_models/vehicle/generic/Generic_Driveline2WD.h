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
// Generic 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef GENERIC_DRIVELINE_2WD_H
#define GENERIC_DRIVELINE_2WD_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Shafts-based 2-WD driveline model for a generic vehicle.
class CH_MODELS_API Generic_Driveline2WD : public ChShaftsDriveline2WD {
  public:
    Generic_Driveline2WD(const std::string& name);
    ~Generic_Driveline2WD() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetDifferentialBoxInertia() const override { return m_differentialbox_inertia; }

    virtual double GetConicalGearRatio() const override { return m_conicalgear_ratio; }

    virtual double GetAxleDifferentialLockingLimit() const override { return m_axle_differential_locking_limit; }

  private:
    static const double m_driveshaft_inertia;
    static const double m_differentialbox_inertia;
    static const double m_conicalgear_ratio;
    static const double m_axle_differential_locking_limit;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
