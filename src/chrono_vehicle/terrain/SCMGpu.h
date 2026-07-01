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
// C++ API for the optional external scm_gpu_core HIP library.
// =============================================================================

#ifndef SCM_GPU_H
#define SCM_GPU_H

#include <cstddef>

#include "chrono_vehicle/terrain/SCMGpuTypes.h"

struct ScmGpuContext;

#ifdef __cplusplus
extern "C" {
#endif

/// Minimum hits per step before GPU path is used (`CHRONO_SCM_GPU_MIN_HITS`, default 8192).
std::size_t scm_gpu_min_hits(void);

/// Buffer capacity to pre-allocate (`CHRONO_SCM_GPU_RESERVE`, default 65536).
std::size_t scm_gpu_reserve_hits(void);

/// Async copy/compute streams (`CHRONO_SCM_GPU_ASYNC`, default on).
int scm_gpu_async_enabled(void);

ScmGpuContext* scm_gpu_create(int device_id);
void scm_gpu_destroy(ScmGpuContext* ctx);
void scm_gpu_reserve(ScmGpuContext* ctx, std::size_t n_hits);
void scm_gpu_warmup(ScmGpuContext* ctx);

chrono::vehicle::scm::gpu::HitInput* scm_gpu_prepare_input(ScmGpuContext* ctx, std::size_t n_hits);
chrono::vehicle::scm::gpu::HitOutput* scm_gpu_prepare_output(ScmGpuContext* ctx, std::size_t n_hits);
chrono::vehicle::scm::gpu::BodyForceAccum* scm_gpu_prepare_body_forces(ScmGpuContext* ctx, std::size_t n_bodies);

int scm_gpu_compute_forces_staged(ScmGpuContext* ctx, const chrono::vehicle::scm::gpu::SoilParams& soil, std::size_t n_hits, std::size_t n_bodies);

void scm_gpu_sync(ScmGpuContext* ctx);

void scm_cpu_compute_forces(const chrono::vehicle::scm::gpu::SoilParams& soil,
                            const chrono::vehicle::scm::gpu::HitInput* in,
                            chrono::vehicle::scm::gpu::HitOutput* out,
                            std::size_t n_hits);

void scm_cpu_reduce_body_forces(const chrono::vehicle::scm::gpu::HitInput* in,
                                const chrono::vehicle::scm::gpu::HitOutput* out,
                                std::size_t n_hits,
                                chrono::vehicle::scm::gpu::BodyForceAccum* bodies,
                                std::size_t n_bodies);

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
