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

#ifndef CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H

#include "chrono_vehicle/ChVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Customized Chrono Irrlicht visualization system for wheeled vehicle simulation.
class CH_VEHICLE_API ChWheeledVehicleVisualSystemVSG : public ChVehicleVisualSystemVSG {
  public:
    /// Construct a wheeled vehicle Irrlicht visualization.
    ChWheeledVehicleVisualSystemVSG();

    ~ChWheeledVehicleVisualSystemVSG() {}

    /// Attach a vehicle to this VSG wheeled vehicle visualization system.
    virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;

    virtual int GetNumDrivenAxles() override;
    virtual double GetTireTorque(int axle, int side) override;

  private:
    ChWheeledVehicle* m_wvehicle;
    int m_drivenAxles = 0;
};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif
