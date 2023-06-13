// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// VSG-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs.
// =============================================================================

#ifndef CH_INTERACTIVE_DRIVER_VSG_H
#define CH_INTERACTIVE_DRIVER_VSG_H

#include <string>

#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/ChVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// VSG-based interactive driver for the a vehicle.
/// This class implements the functionality required by the base ChInteractiveDriver class using keyboard inputs.
class CH_VEHICLE_API ChInteractiveDriverVSG : public ChInteractiveDriver {
  public:
    ChInteractiveDriverVSG(ChVehicleVisualSystemVSG& vsys);
    ~ChInteractiveDriverVSG();

    /// Initialize this driver system.
    virtual void Initialize() override;

    /// Increase Throttle
    void IncreaseThrottle();

    /// Decrease Throttle
    void DecreaseThrottle();

    /// Steering Left
    void SteeringLeft();

    /// Steering Right
    void SteeringRight();

    /// Increase Clutch
    void IncreaseClutch();

    /// Decrease Clutch
    void DecreaseClutch();

    /// Center Steering
    void SteeringCenter();

    /// Release Pedals
    void ReleasePedals();

    friend class ChVehicleVisualSystemVSG;
};

/// @} vehicle_driver

}  // namespace vehicle
}  // namespace chrono

#endif
