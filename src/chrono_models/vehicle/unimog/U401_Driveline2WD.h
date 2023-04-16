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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// UAZBUS 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef U401_DRIVELINE_2WD_H
#define U401_DRIVELINE_2WD_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace unimog {

/// @addtogroup vehicle_models_401
/// @{

/// Shafts-based 2-WD driveline for the UAZBUS vehicle.
class CH_MODELS_API U401_Driveline2WD : public ChShaftsDriveline2WD {
   public:
    U401_Driveline2WD(const std::string& name);

    ~U401_Driveline2WD() {}

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

/// @} vehicle_models_U401

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono

#endif
