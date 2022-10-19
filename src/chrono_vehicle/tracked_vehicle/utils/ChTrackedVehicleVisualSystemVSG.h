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
// Irrlicht-based visualization for wheeled vehicles.
// This class extends ChVehicleVisualSystemIrrlicht.
//
// =============================================================================

#ifndef CH_TRACKED_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_TRACKED_VEHICLE_VISUAL_SYSTEM_VSG_H

#include "chrono_vehicle/utils/ChVehicleVisualSystemVSG.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_utils
/// @{

/// Customized Chrono Irrlicht visualization system for wheeled vehicle simulation.
class CH_VEHICLE_API ChTrackedVehicleVisualSystemVSG : public ChVehicleVisualSystemVSG {
  public:
    /// Construct a wheeled vehicle Irrlicht visualization.
    ChTrackedVehicleVisualSystemVSG();

    ~ChTrackedVehicleVisualSystemVSG() {}

    /// Attach a vehicle to this VSG wheeled vehicle visualization system.
    virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;

    virtual double GetSprocketTorque(int side) override;
    virtual double GetSprocketSpeed(int side) override;

private:
    ChTrackedVehicle* m_tvehicle;
    int m_drivenAxles = 0;
};

/// @} vehicle_tracked_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
