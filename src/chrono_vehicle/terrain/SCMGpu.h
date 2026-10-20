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

#ifdef __cplusplus
extern "C" {
#endif

/// Return 1 if a usable GPU device is present, 0 otherwise. Probed once per process; a negative
/// result is reported once on stderr. scm_gpu_create and scm_raycast_gpu_create return nullptr in
/// that case, and SCMTerrain then uses its CPU paths.
int scm_gpu_device_available(void);

ScmGpuContext* scm_gpu_create(int device_id);
void scm_gpu_destroy(ScmGpuContext* ctx);
void scm_gpu_reserve(ScmGpuContext* ctx, std::size_t n_hits);
/// Allocate buffers for at least reserve_hits hits and run a one-hit launch (async: use the pipelined path).
void scm_gpu_warmup(ScmGpuContext* ctx, std::size_t reserve_hits, int async);

chrono::vehicle::scm::gpu::HitInput* scm_gpu_prepare_input(ScmGpuContext* ctx, std::size_t n_hits);
chrono::vehicle::scm::gpu::HitOutput* scm_gpu_prepare_output(ScmGpuContext* ctx, std::size_t n_hits);
chrono::vehicle::scm::gpu::BodyForceAccum* scm_gpu_prepare_body_forces(ScmGpuContext* ctx, std::size_t n_bodies);

int scm_gpu_compute_forces_staged(ScmGpuContext* ctx, const chrono::vehicle::scm::gpu::SoilParams& soil, std::size_t n_hits, std::size_t n_bodies, int async);

void scm_gpu_sync(ScmGpuContext* ctx);

int scm_gpu_compute_forces(ScmGpuContext* ctx,
                           const chrono::vehicle::scm::gpu::SoilParams& soil,
                           const chrono::vehicle::scm::gpu::HitInput* in,
                           chrono::vehicle::scm::gpu::HitOutput* out_host,
                           std::size_t n_hits,
                           std::size_t n_bodies,
                           int async);

#ifdef __cplusplus
}
#endif

#endif
