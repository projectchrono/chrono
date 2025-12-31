// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// VSG-based visualization for a suspension test rig.
//
// =============================================================================

#ifndef CH_STR_VISUAL_SYSTEM_VSG_H
#define CH_STR_VISUAL_SYSTEM_VSG_H

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Customized Chrono::VSG visualization system for suspension test rig simulation.
class CH_VEHICLE_API ChSuspensionTestRigVisualSystemVSG : public vsg3d::ChVisualSystemVSG {
  public:
    /// Construct a suspension test rig VSG visualization.
    ChSuspensionTestRigVisualSystemVSG();

    ~ChSuspensionTestRigVisualSystemVSG() {}

    /// Initialize the visualization system.
    virtual void Initialize() override;

    /// Attach a suspension test rig to this VSG visualization system.
    void AttachSTR(ChSuspensionTestRig* rig);

  private:
    ChSuspensionTestRig* m_rig;

    bool m_chassis_visible;
    bool m_suspension_visible;
    bool m_steering_visible;
    bool m_wheel_visible;
    bool m_tire_visible;

    friend class ChSTRGuiComponentVSG;
    friend class ChSTRKeyboardHandlerVSG;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
