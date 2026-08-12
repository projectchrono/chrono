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
// SCM GPU ray-cast batch types (SoA-friendly POD layout for the HIP ray-cast backend).
// Mirrors SCMGpuTypes.h's conventions.
// =============================================================================

#ifndef SCM_RAYCAST_GPU_TYPES_H
#define SCM_RAYCAST_GPU_TYPES_H

#include <cstddef>
#include <cstdint>

namespace chrono {
namespace vehicle {
namespace scm {
namespace gpu {

/// Local-space (body-relative) mesh vertex, shared across all candidate bodies' meshes in one buffer.
struct RaycastVertex {
    double x, y, z;
};

/// Local-space mesh face: indices are absolute into the shared vertex buffer (host applies each body's
/// vertex offset before upload); body_slot selects which transform in RaycastBodyTransform[] to apply.
struct RaycastFace {
    int32_t i0, i1, i2;
    int32_t body_slot;
};

/// Per-body absolute world transform (position + row-major 3x3 rotation), one per body_slot.
struct RaycastBodyTransform {
    double px, py, pz;
    double r00, r01, r02;
    double r10, r11, r12;
    double r20, r21, r22;
};

/// Per-body margin (collision envelope + swept-sphere radius), one per body_slot.
/// Applied to the winning hit's point along its (outward-oriented) triangle normal.
struct RaycastBodyMargin {
    double margin;
};

/// One SCM grid-node ray query (already RayOBBtest-prefiltered and compacted on the host).
struct RaycastQuery {
    double from_x, from_y, from_z;
    double to_x, to_y, to_z;
};

/// Result for one RaycastQuery (same index).
struct RaycastResult {
    int32_t hit;
    int32_t body_slot;
    double hit_x, hit_y, hit_z;
};

}  // namespace gpu
}  // namespace scm
}  // namespace vehicle
}  // namespace chrono

#endif
