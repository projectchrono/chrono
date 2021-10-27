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
// mrole 8WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef MROLE_DRIVELINE_8WD_H
#define MROLE_DRIVELINE_8WD_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline8WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// Shafts-based 8-WD driveline for the mrole vehicle.
class CH_MODELS_API mrole_Driveline8WD : public ChShaftsDriveline8WD {
  public:
    mrole_Driveline8WD(const std::string& name);
    ~mrole_Driveline8WD() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetGroupDiffInputShaftInertia() const override { return m_group_inshaft_inertia; }
    virtual double GetAxleDiffInputShaftInertia() const override { return m_axle_inshaft_inertia; }
    virtual double GetAxleDiffBoxInertia() const override { return m_axle_diffbox_inertia; }

    virtual double GetAxleDiffConicalGearRatio() const override { return m_axle_conicalgear_ratio; }

    virtual double GetCentralDifferentialLockingLimit() const override { return m_central_differential_locking_limit; }
    virtual double GetGroupDifferentialLockingLimit() const override { return m_group_differential_locking_limit; }
    virtual double GetAxleDifferentialLockingLimit() const override { return m_axle_differential_locking_limit; }

  private:
    // Shaft inertias.
    static const double m_driveshaft_inertia;
    static const double m_group_inshaft_inertia;
    static const double m_axle_inshaft_inertia;
    static const double m_axle_diffbox_inertia;

    // Gear ratios.
    static const double m_axle_conicalgear_ratio;

    // Differential locking torque limits.
    static const double m_central_differential_locking_limit;
    static const double m_group_differential_locking_limit;
    static const double m_axle_differential_locking_limit;
};

/// @} vehicle_models_mrole

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
