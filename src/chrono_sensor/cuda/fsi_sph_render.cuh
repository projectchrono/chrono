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
// CUDA helpers for native Chrono::FSI::SPH rendering in Chrono::Sensor.
//
// =============================================================================

#ifndef CH_SENSOR_FSI_SPH_RENDER_CUH
#define CH_SENSOR_FSI_SPH_RENDER_CUH

#include <cstddef>

#include <cuda_runtime.h>
#include <optix.h>

#include "chrono/core/ChVector3.h"
#include "chrono_fsi/sph/ChFsiDataTypesSPH.h"

namespace chrono {
namespace sensor {

void cuda_update_fsi_sph_sprite_instances(const chrono::fsi::sph::Real4* pos_rad,
                                          size_t source_count,
                                          size_t render_count,
                                          OptixInstance* instances,
                                          const OptixTraversableHandle* gas_handles,
                                          const unsigned int* sbt_offsets,
                                          const float3* template_scales,
                                          unsigned int num_templates,
                                          float render_particle_spacing,
                                          const ChVector3f& position_jitter,
                                          const ChVector3f& origin_offset);

}  // namespace sensor
}  // namespace chrono

#endif
