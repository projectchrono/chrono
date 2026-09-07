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
// HIP host/device API for the SCM ray-cast backend (built into Chrono_vehicle).
//
// Deliberately synchronous / unpipelined for this first port (v1): correctness over throughput.
// The contact-force backend (SCMGpu.h) uses an async double-buffered pattern -- worth mirroring
// here later if profiling shows launch/copy overhead matters at production query counts.
//
// Mesh geometry and per-body transforms are uploaded through SEPARATE calls: mesh (vertices/faces/
// margins) only changes when the candidate body set changes (rare -- never, in the common
// single-vehicle case), while transforms change every step. The caller (SCMTerrainRaycastGpu.cpp)
// caches the candidate set and only calls scm_raycast_gpu_upload_mesh when it actually changes.
// =============================================================================

#ifndef SCM_RAYCAST_GPU_H
#define SCM_RAYCAST_GPU_H

#include <cstddef>

#include "chrono_vehicle/terrain/SCMRaycastGpuTypes.h"

struct ScmRaycastGpuContext;

/// Floating-point precision used by the kernel for a given context. kFP32 is the default on every
/// platform so that a model behaves identically on AMD and NVIDIA; kFP64 is available for validation
/// or for parts with the double-precision throughput to spare. Selected in SCMTerrainRaycastGpu.cpp
/// and overridable with env SCM_RAYCAST_GPU_PRECISION=fp32|fp64.
enum class ScmRaycastGpuPrecision {
    kFP64 = 0,
    kFP32 = 1,
};

#ifdef __cplusplus
extern "C" {
#endif

ScmRaycastGpuContext* scm_raycast_gpu_create(int device_id, ScmRaycastGpuPrecision precision);
void scm_raycast_gpu_destroy(ScmRaycastGpuContext* ctx);

/// Upload mesh geometry (local-space, shared vertex/face buffers across all candidate bodies) and
/// per-body margins. Call only when the candidate body set (or its geometry) has actually changed.
int scm_raycast_gpu_upload_mesh(ScmRaycastGpuContext* ctx,
                                const chrono::vehicle::scm::gpu::RaycastVertex* verts,
                                int n_verts,
                                const chrono::vehicle::scm::gpu::RaycastFace* faces,
                                int n_faces,
                                const chrono::vehicle::scm::gpu::RaycastBodyMargin* margins,
                                int n_bodies);

/// Upload per-body transforms only. Call every step -- this is the only per-step upload needed once
/// scm_raycast_gpu_upload_mesh has been called at least once for the current candidate set.
int scm_raycast_gpu_upload_transforms(ScmRaycastGpuContext* ctx,
                                      const chrono::vehicle::scm::gpu::RaycastBodyTransform* xforms,
                                      int n_bodies);

/// Run the ray-cast kernel for n_queries queries against the geometry uploaded via
/// scm_raycast_gpu_upload_mesh()/scm_raycast_gpu_upload_transforms(), writing n_queries results to
/// out_results (host pointer).
int scm_raycast_gpu_run(ScmRaycastGpuContext* ctx,
                        const chrono::vehicle::scm::gpu::RaycastQuery* queries,
                        chrono::vehicle::scm::gpu::RaycastResult* out_results,
                        int n_queries);

#ifdef __cplusplus
}
#endif

#endif
