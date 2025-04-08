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
#include "chrono_fsi/sph/utils/FsiProblemRelocate.cuh"

namespace chrono {
namespace fsi {
namespace sph {

// -----------------------------------------------------------------------------

// RelocateFunction to shift marker position by a given vector.
struct shift_op : public FsiDataManager::RelocateFunction {
    shift_op(const Real3& shift) : s(shift) {}
    __device__ virtual void operator()(Real3& pos) const override { pos += s; }
    Real3 s;
};

void shiftBCE(const Real3& shift, FsiDataManager& dm) {
    dm.Shift(MarkerType::BCE_WALL, shift);
    ////dm.Relocate(MarkerType::BCE_WALL, shift_op(shift));
}

void shiftSPH(const Real3& shift, FsiDataManager& dm) {
    dm.Shift(MarkerType::SPH_PARTICLE, shift);
}

// -----------------------------------------------------------------------------

// SelectorFunction to find particles in a given AABB.
struct inaabb_op : public FsiDataManager::SelectorFunction {
    inaabb_op(const Real3& aabb_min, const Real3& aabb_max) : min(aabb_min), max(aabb_max) {}
    __device__ virtual bool operator()(const Real3& pos) const override {
        if (pos.x < min.x || pos.x > max.x)
            return false;
        if (pos.y < min.y || pos.y > max.y)
            return false;
        if (pos.z < min.z || pos.z > max.z)
            return false;
        return true;
    }
    Real3 min;
    Real3 max;
};

// RelocateFunction to move particle to a given AABB.
struct toaabb_op : public FsiDataManager::RelocateFunction {
    toaabb_op(const Real3& aabb_min, const Real3& aabb_max, Real spacing)
        : min(aabb_min), max(aabb_max), delta(spacing) {}
    __device__ virtual void operator()(Real3& pos) const override {
        //// TODO
        pos += mR3(0, 2, 0);  // testing...
    }
    Real3 min;
    Real3 max;
    Real delta;
};

void moveSPH(const Real3& aabb_src_min,
             const Real3& aabb_src_max,
             const Real3& aabb_dest_min,
             const Real3& aabb_dest_max,
             Real spacing,
             FsiDataManager& dm) {
    dm.MoveAABB(MarkerType::SPH_PARTICLE, aabb_src_min, aabb_src_max, aabb_dest_min, aabb_dest_max, spacing);
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
