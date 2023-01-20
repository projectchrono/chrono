// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// VSG-based GUI driver for the a track test rig.
// This class implements the functionality required by its base class using
// keyboard inputs.
//
// =============================================================================

#ifndef CH_TTR_INTERACTIVE_DRIVER_VSG_H
#define CH_TTR_INTERACTIVE_DRIVER_VSG_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystemVSG.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// VSG-based GUI driver for the a track test rig.
/// This class implements the functionality required by its base class using keyboard inputs.
class CH_VEHICLE_API ChTrackTestRigInteractiveDriverVSG : public ChTrackTestRigDriver {
  public:
    ChTrackTestRigInteractiveDriverVSG(vsg3d::ChVisualSystemVSG& vsys);

    ~ChTrackTestRigInteractiveDriverVSG() {}

    void SetThrottleDelta(double delta) { m_throttleDelta = delta; }
    void SetDisplacementDelta(double delta) { m_displDelta = delta; }

    //// TODO

  private:
    virtual std::string GetInfoMessage() const override { return m_msg; }

    double m_throttleDelta;
    double m_displDelta;
    int m_current_post;
    std::string m_msg;
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
