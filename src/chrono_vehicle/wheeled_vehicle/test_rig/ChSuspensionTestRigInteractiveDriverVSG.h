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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// VSG-based GUI driver for the a suspension test rig.
// This class implements the functionality required by its base class using
// keyboard inputs.
//
// =============================================================================

#ifndef CH_STR_INTERACTIVE_DRIVER_VSG_H
#define CH_STR_INTERACTIVE_DRIVER_VSG_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// VSG-based GUI driver for the a suspension test rig.
/// This class implements the functionality required by its base class using keyboard inputs.
class CH_VEHICLE_API ChSuspensionTestRigInteractiveDriverVSG : public ChSuspensionTestRigDriver {
  public:
    ChSuspensionTestRigInteractiveDriverVSG(vsg3d::ChVisualSystemVSG& vsys);

    ~ChSuspensionTestRigInteractiveDriverVSG() {}

    //// TODO

  private:
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
