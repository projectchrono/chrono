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
// VSG-based visualization wrapper for track test rig.
//
// =============================================================================

#ifndef CH_TTR_VISUAL_SYSTEM_VSG_H
#define CH_TTR_VISUAL_SYSTEM_VSG_H

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// Customized Chrono::VSG visualization system for track test rig simulation.
class CH_VEHICLE_API ChTrackTestRigVisualSystemVSG : public vsg3d::ChVisualSystemVSG {
  public:
    ChTrackTestRigVisualSystemVSG();

    ~ChTrackTestRigVisualSystemVSG() {}

    /// Initialize the visualization system.
    virtual void Initialize() override;

    /// Attach a suspension test rig to this VSG visualization system.
    void AttachTTR(ChTrackTestRig* rig);

  private:
    ChTrackTestRig* m_rig;

    bool m_chassis_visible;
    bool m_sprocket_visible;
    bool m_idler_visible;
    bool m_suspension_visible;
    bool m_shoe_visible;
    bool m_wheel_visible;

    friend class ChTTRGuiComponentVSG;
    friend class ChTTRKeyboardHandlerVSG;
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
