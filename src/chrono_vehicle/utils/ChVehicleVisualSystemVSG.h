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
// Irrlicht-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemIrrlicht and provides the following functionality:
//   - rendering of the entire Irrlicht scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#ifndef CH_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_VEHICLE_VISUAL_SYSTEM_VSG_H

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_utils
/// @{

/// Customized Chrono Irrlicht visualization for vehicle simulations.
/// This class implements an Irrlicht-based visualization wrapper for vehicles.
/// This class extends ChVisualSystemIrrlicht and provides the following functionality:
///   - rendering of the entire Irrlicht scene
///   - custom chase-camera (which can be controlled with keyboard)
///   - optional rendering of links, springs, stats, etc.
class CH_VEHICLE_API ChVehicleVisualSystemVSG : public ChVehicleVisualSystem, public vsg3d::ChVisualSystemVSG {
  public:
    /// Construct a vehicle Irrlicht visualization system
    ChVehicleVisualSystemVSG();
    virtual void Initialize() override;
    /// Update information related to driver inputs.
    virtual void Synchronize(const std::string& msg, const DriverInputs& driver_inputs) override;
    /// Advance the dynamics of the chase camera.
    /// The integration of the underlying ODEs is performed using as many steps as needed to advance
    /// by the specified duration.
    void Advance(double step);
};

// @} vehicle_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
