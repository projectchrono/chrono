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

#ifndef CH_WHEELED_VEHICLE_VISUAL_SYSTEM_IRRLICHT_H
#define CH_WHEELED_VEHICLE_VISUAL_SYSTEM_IRRLICHT_H

#include "chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_utils
/// @{

/// Customized Chrono Irrlicht visualization system for wheeled vehicle simulation.
class CH_VEHICLE_API ChWheeledVehicleVisualSystemIrrlicht : public ChVehicleVisualSystemIrrlicht {
  public:
    /// Construct a wheeled vehicle Irrlicht visualization.
    ChWheeledVehicleVisualSystemIrrlicht();

    ~ChWheeledVehicleVisualSystemIrrlicht() {}

    /// Attach a vehicle to this Irrlicht wheeled vehicle visualization system.
    virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;

  private:
    virtual void renderOtherStats(int left, int top) override;

    ChWheeledVehicle* m_wvehicle;
};

/// @} vehicle_wheeled_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
