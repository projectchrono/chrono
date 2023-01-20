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
// VSG-based visualization for wheeled vehicles.
//
// =============================================================================

#ifndef CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H

#include "chrono_vehicle/ChVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_vis
/// @{

/// Customized Chrono::VSG visualization system for wheeled vehicle simulation.
class CH_VEHICLE_API ChWheeledVehicleVisualSystemVSG : public ChVehicleVisualSystemVSG {
  public:
    /// Construct a wheeled vehicle Irrlicht visualization.
    ChWheeledVehicleVisualSystemVSG();

    ~ChWheeledVehicleVisualSystemVSG() {}

    /// Attach a vehicle to this VSG wheeled vehicle visualization system.
    virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;

    virtual void AppendGUIStats() override;

  private:
    ChWheeledVehicle* m_wvehicle;
};

/// @} vehicle_vis

}  // end namespace vehicle
}  // end namespace chrono

#endif
