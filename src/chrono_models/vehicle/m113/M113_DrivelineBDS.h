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
// Authors:
// =============================================================================
//
// M113 driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef M113_DRIVELINE_BDS_H
#define M113_DRIVELINE_BDS_H

#include "chrono_vehicle/tracked_vehicle/driveline/ChTrackDrivelineBDS.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Shafts-based driveline model for the M113 vehicle.
class CH_MODELS_API M113_DrivelineBDS : public ChTrackDrivelineBDS {
  public:
    M113_DrivelineBDS();

    ~M113_DrivelineBDS() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetDifferentialBoxInertia() const override { return m_differentialbox_inertia; }

    virtual double GetConicalGearRatio() const override { return m_conicalgear_ratio; }

    virtual double GetDifferentialLockingLimit() const override { return m_differential_locking_limit; }

  private:
    // Shaft inertias
    static const double m_driveshaft_inertia;
    static const double m_differentialbox_inertia;

    // Gear ratio
    static const double m_conicalgear_ratio;

    // Differential locking torque limit.
    static const double m_differential_locking_limit;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
