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
// HIP host/device API for the SCM contact-force backend (built into Chrono_vehicle).
// =============================================================================

#ifndef SCM_GPU_H
#define SCM_GPU_H

#include <cstddef>

#include "chrono_vehicle/terrain/SCMGpuTypes.h"

struct ScmGpuContext;

namespace chrono {
namespace vehicle {
namespace scm_gpu {

/// Runtime tuning for the SCM GPU backend (defaults are suitable for production).
struct Config {
    bool enabled = true;              ///< use GPU when eligibility checks pass
    std::size_t min_hits = 8192;      ///< CPU fallback below this hit count
    std::size_t reserve_hits = 65536; ///< pre-allocated hit buffer capacity
    bool async = true;                ///< async HIP streams for pack/compute/scatter
    bool profile = false;             ///< log pack/gpu/scatter timings to stderr
};

void SetConfig(const Config& config);
Config GetConfig();

}  // namespace scm_gpu
}  // namespace vehicle
}  // namespace chrono

#ifdef __cplusplus
extern "C" {
#endif

std::size_t scm_gpu_min_hits(void);
std::size_t scm_gpu_reserve_hits(void);
int scm_gpu_async_enabled(void);

ScmGpuContext* scm_gpu_create(int device_id);
void scm_gpu_destroy(ScmGpuContext* ctx);
void scm_gpu_reserve(ScmGpuContext* ctx, std::size_t n_hits);
void scm_gpu_warmup(ScmGpuContext* ctx);

chrono::vehicle::scm::gpu::HitInput* scm_gpu_prepare_input(ScmGpuContext* ctx, std::size_t n_hits);
chrono::vehicle::scm::gpu::HitOutput* scm_gpu_prepare_output(ScmGpuContext* ctx, std::size_t n_hits);
chrono::vehicle::scm::gpu::BodyForceAccum* scm_gpu_prepare_body_forces(ScmGpuContext* ctx, std::size_t n_bodies);

int scm_gpu_compute_forces_staged(ScmGpuContext* ctx,
                                  const chrono::vehicle::scm::gpu::SoilParams& soil,
                                  std::size_t n_hits,
                                  std::size_t n_bodies);

void scm_gpu_sync(ScmGpuContext* ctx);

int scm_gpu_compute_forces(ScmGpuContext* ctx,
                           const chrono::vehicle::scm::gpu::SoilParams& soil,
                           const chrono::vehicle::scm::gpu::HitInput* in,
                           const chrono::vehicle::scm::gpu::HitOutput* out_host,
                           std::size_t n_hits,
                           std::size_t n_bodies);

#ifdef __cplusplus
}
#endif

#endif
