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
// SCM GPU batch types (SoA layout for contact-force HIP backend).
// =============================================================================

#ifndef SCM_GPU_TYPES_H
#define SCM_GPU_TYPES_H

#include <cstddef>
#include <cstdint>

namespace chrono {
namespace vehicle {
namespace scm {
namespace gpu {

/// Global soil parameters (one per step).
struct SoilParams {
    double bekker_kphi;
    double bekker_kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_mu;
    double janosi_shear;
    double elastic_k;
    double damping_r;
    double area;
    double dt;
};

/// Per-hit inputs (packed on host after ray cast + patch clustering).
struct HitInput {
    double level_initial;
    double level;
    double sinkage_plastic;
    double kshear;
    double sigma_yield;
    double normal_z;
    double hit_level;
    double patch_oob;
    double vn;
    double vt;
    double nx, ny, nz;
    double tx, ty, tz;
    double px, py, pz;
    double bx, by, bz;
    double c_cohesion;
    double c_mu;
    double c_janosi;
    double area_ratio;
    int32_t body_id;
    int32_t grid_ix;
    int32_t grid_iy;
    int32_t active;
};

/// Per-hit outputs (node update + force contribution).
struct HitOutput {
    double level;
    double sinkage;
    double sinkage_plastic;
    double sinkage_elastic;
    double sigma;
    double sigma_yield;
    double kshear;
    double tau;
    double step_plastic_flow;
    double fn_x, fn_y, fn_z;
    double ft_x, ft_y, ft_z;
    int32_t active;
};

/// Per-body force/moment accumulator (device reduce → pinned host).
struct BodyForceAccum {
    double fx, fy, fz;
    double mx, my, mz;
};

}  // namespace gpu
}  // namespace scm
}  // namespace vehicle
}  // namespace chrono

#endif
