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
// Authors: Radu Serban
// =============================================================================
//
// OpenGL-based visualization wrapper for vehicles.
//
// =============================================================================

#ifndef CH_VEHICLE_VISUAL_SYSTEM_OPENGL_H
#define CH_VEHICLE_VISUAL_SYSTEM_OPENGL_H

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_vis
/// @{

/// OpenGL-based Chrono run-time visualization system.
class CH_VEHICLE_API ChVehicleVisualSystemOpenGL : public ChVehicleVisualSystem, public opengl::ChVisualSystemOpenGL {
  public:
    /// Construct a vehicle OpenGL visualization system
    ChVehicleVisualSystemOpenGL();

    virtual ~ChVehicleVisualSystemOpenGL();

    /// Initialize the visualization system.
    virtual void Initialize() override;

    /// Advance the dynamics of the chase camera.
    /// The integration of the underlying ODEs is performed using as many steps as needed to advance
    /// by the specified duration.
    virtual void Advance(double step) override;

  protected:
    friend class ChVehicleKeyboardHandlerOpenGL;
};

// @} vehicle_vis

}  // namespace vehicle
}  // namespace chrono

#endif
