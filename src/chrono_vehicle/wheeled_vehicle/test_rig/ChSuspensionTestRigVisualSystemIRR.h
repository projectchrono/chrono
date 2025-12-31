// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// Irrlicht-based visualization for a suspension test rig.
//
// =============================================================================

#ifndef CH_STR_VISUAL_SYSTEM_IRR_H
#define CH_STR_VISUAL_SYSTEM_IRR_H

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

namespace chrono {
namespace vehicle {

class ChSTRKeyboardHandlerIRR;

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Customized Chrono::Irrlicht visualization system for suspension test rig simulation.
class CH_VEHICLE_API ChSuspensionTestRigVisualSystemIRR : public irrlicht::ChVisualSystemIrrlicht {
  public:
    /// Construct a wheeled vehicle Irrlicht visualization.
    ChSuspensionTestRigVisualSystemIRR();

    ~ChSuspensionTestRigVisualSystemIRR() {}

    /// Attach a suspension test rig to this VSG visualization system.
    void AttachSTR(ChSuspensionTestRig* rig);

    /// Initialize the visualizatin system.
    virtual void Initialize() override;

  private:
    bool m_irr_initialized = false;
    std::shared_ptr<ChSTRKeyboardHandlerIRR> m_keyboard_handler;
    ChSuspensionTestRig* m_rig;

    friend class ChSTRKeyboardHandlerIRR;
};

/// @} vehicle_wheeled_test_rig

class CH_VEHICLE_API ChSTRKeyboardHandlerIRR : public irr::IEventReceiver {
  public:
    ChSTRKeyboardHandlerIRR(ChSuspensionTestRigVisualSystemIRR* app);
    bool OnEvent(const irr::SEvent& event) override;

  private:
    ChSuspensionTestRigVisualSystemIRR* m_app;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
