// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Device ulities for moving SPH particles and BCE markers external to the solver
//
// =============================================================================

////#define DEBUG_LOG

#include <cstdio>

#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/transform.h>

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/utils/UtilsDevice.cuh"
#include "chrono_fsi/sph/utils/Relocate.cuh"

namespace chrono {
namespace fsi {
namespace sph {

// Implementation of a particle RelocateFunction that shifts the particle position
struct shift_op : public FsiDataManager::RelocateFunction {
    shift_op(Real3 shift_dist) : dist(shift_dist) {}
    __host__ __device__ virtual Real3 operator()(Real3& x) const override {
        return x + dist;
    }
    Real3 dist;
};

void shiftBCE(const Real3& shift_dist, double spacing, FsiDataManager& dm) {
    dm.Shift(MarkerType::BCE_WALL, shift_op(shift_dist));
}

void shiftSPH(const Real3& shift_dist, double spacing, FsiDataManager& dm) {
    dm.Shift(MarkerType::SPH_PARTICLE, shift_op(shift_dist));
}

// -----------------------------------------------------------------------------

struct move_op : public FsiDataManager::RelocateFunction {
    move_op(const Real3& aabb_min, const Real3& aabb_max, double spacing)
        : min(aabb_min), max(aabb_max), delta(spacing) {}
    __host__ __device__ virtual Real3 operator()(Real3& x) const override {
        //// TODO
        return x;
    }
    Real3 min;
    Real3 max;
    Real delta;
};

void moveSPH(const FsiDataManager::SelectorFunction& op_select,
             const Real3& aabb_min,
             const Real3& aabb_max,
             double spacing,
             FsiDataManager& dm) {
    dm.Move(MarkerType::SPH_PARTICLE, op_select, move_op(aabb_min, aabb_max, spacing));
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
