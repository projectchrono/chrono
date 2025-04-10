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

void shift_BCE(const Real3& shift, const FsiDataManager::DefaultProperties& props, FsiDataManager& dm) {
    dm.Shift(MarkerType::BCE_WALL, shift, props);
    ////dm.Relocate(MarkerType::BCE_WALL, shift_op(shift), props);
}

void shift_SPH(const Real3& shift, const FsiDataManager::DefaultProperties& props, FsiDataManager& dm) {
    dm.Shift(MarkerType::SPH_PARTICLE, shift, props);
    ////dm.Relocate(MarkerType::SPH_PARTICLE, shift_op(shift), props);
}

// -----------------------------------------------------------------------------

// SelectorFunction to find particles in a given AABB.
struct inaabb_op : public FsiDataManager::SelectorFunction {
    inaabb_op(const RealAABB& aabb_src) : aabb(aabb_src) {}
    __device__ virtual bool operator()(const Real3& pos) const override {
        if (pos.x < aabb.min.x || pos.x > aabb.max.x)
            return false;
        if (pos.y < aabb.min.y || pos.y > aabb.max.y)
            return false;
        if (pos.z < aabb.min.z || pos.z > aabb.max.z)
            return false;
        return true;
    }
    RealAABB aabb;
};

// RelocateFunction to move particle to a given AABB.
struct toaabb_op : public FsiDataManager::RelocateFunction {
    toaabb_op(const RealAABB& aabb_dest, Real spacing)
        : aabb(aabb_dest), delta(spacing) {}
    __device__ virtual void operator()(Real3& pos) const override {
        pos += mR3(0, 2, 0);  // testing... //// TODO
    }
    RealAABB aabb;
    Real delta;
};

void moveAABB2AABB_SPH(const RealAABB& aabb_src,
                       const RealAABB& aabb_dest,
                       Real spacing,
                       const FsiDataManager::DefaultProperties& props,
                       FsiDataManager& dm) {
    dm.MoveAABB2AABB(MarkerType::SPH_PARTICLE, aabb_src, aabb_dest, spacing, props);
    ////dm.Relocate(MarkerType::SPH_PARTICLE, toaabb_op(aabb_dest, spacing), inaabb_op(aabb_src), props);
}

void moveAABB2AABB_SPH(const RealAABB& aabb_src,
                       const IntAABB& aabb_dest,
                       Real spacing,
                       const FsiDataManager::DefaultProperties& props,
                       FsiDataManager& dm) {
    dm.MoveAABB2AABB(MarkerType::SPH_PARTICLE, aabb_src, aabb_dest, spacing, props);
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
