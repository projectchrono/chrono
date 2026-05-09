// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Bocheng Zou
// =============================================================================
//
// Native Chrono::FSI::SPH rendering options for Chrono::Sensor.
//
// =============================================================================

#ifndef CH_FSI_SPH_RENDER_H
#define CH_FSI_SPH_RENDER_H

#include <memory>
#include <vector>

#include "chrono/assets/ChVisualShape.h"
#include "chrono/core/ChVector3.h"
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChConfigSensor.h"

namespace chrono {

namespace fsi {
namespace sph {
class ChFsiFluidSystemSPH;
}
}  // namespace fsi

namespace sensor {

/// Options for rendering Chrono::FSI::SPH fluid markers in Chrono::Sensor.
struct CH_SENSOR_API ChFsiSphRenderOptions {
    std::vector<std::shared_ptr<ChVisualShape>> sprite_shapes;            ///< Required visual shape templates used for marker sprites.
    ChVector3f sprite_position_jitter = ChVector3f(0.005f, 0.005f, 0.f);  ///< Deterministic per-marker sprite jitter.
    float render_particle_spacing = 0.f;  ///< Absolute visual particle spacing. Must be positive to render particles.
};

#ifdef CHRONO_FSI_SPH

/// One FSI-SPH render source attached to a Sensor scene.
struct CH_SENSOR_API ChFsiSphRenderSource {
    int id = -1;
    std::shared_ptr<chrono::fsi::sph::ChFsiFluidSystemSPH> system;
    ChFsiSphRenderOptions options;
};

#endif

}  // namespace sensor
}  // namespace chrono

#endif
