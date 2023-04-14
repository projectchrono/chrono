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
// VSG-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemVSG and provides the following functionality:
//   - rendering of the entire VSG scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#ifndef CH_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_VEHICLE_VISUAL_SYSTEM_VSG_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

class ChInteractiveDriverVSG;

/// @addtogroup vehicle_vis
/// @{

/// VSG-based Chrono run-time visualization system.
class CH_VEHICLE_API ChVehicleVisualSystemVSG : public ChVehicleVisualSystem, public vsg3d::ChVisualSystemVSG {
  public:
    /// Construct a vehicle VSG visualization system
    ChVehicleVisualSystemVSG();

    virtual ~ChVehicleVisualSystemVSG();

    /// Initialize the visualization system.
    virtual void Initialize() override;

    /// Advance the dynamics of the chase camera.
    /// The integration of the underlying ODEs is performed using as many steps as needed to advance
    /// by the specified duration.
    virtual void Advance(double step) override;

  protected:
    virtual void AppendGUIStats() {}

    ChInteractiveDriverVSG* m_driver;
    bool m_has_TC;

    friend class ChInteractiveDriverVSG;
    friend class ChVehicleGuiComponentVSG;
    friend class ChVehicleKeyboardHandlerVSG;
};

// @} vehicle_vis

}  // namespace vehicle
}  // namespace chrono
#endif