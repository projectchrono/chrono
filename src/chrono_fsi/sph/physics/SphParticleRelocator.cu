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

#include <thrust/execution_policy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/count.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/functional.h>
#include <thrust/transform.h>
#include <thrust/partition.h>
#include <thrust/zip_function.h>

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/physics/SphParticleRelocator.cuh"
#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

SphParticleRelocator::SphParticleRelocator(FsiDataManager& data_mgr, const DefaultProperties& props)
    : m_data_mgr(data_mgr), m_props(props) {}

// Relocation function to shift marker position by a given vector.
// Implements a Thrust unary function to be used with thrust::for_each.
struct shift_op {
    shift_op(const Real3& shift, const SphParticleRelocator::DefaultProperties& props) : s(shift), p(props) {}

    template <typename T>
    __device__ void operator()(const T& a) const {
        // Modify position
        Real4 posw = thrust::get<0>(a);
        Real3 pos = mR3(posw);
        pos += s;
        thrust::get<0>(a) = mR4(pos, posw.w);

        // Reset all other marker properties
        Real3 zero = mR3(0);
        thrust::get<1>(a) = zero;                                        // velocity
        thrust::get<2>(a) = mR4(p.rho0, 0, p.mu0, thrust::get<2>(a).w);  // rho, pres, mu, type
        thrust::get<3>(a) = zero;                                        // tau diagonal
        thrust::get<4>(a) = zero;                                        // tau off-diagonal
    }

    Real3 s;
    SphParticleRelocator::DefaultProperties p;
};

void SphParticleRelocator::Shift(MarkerType type, const Real3& shift) {
    // Get start and end indices in marker data vectors based on specified type
    int start_idx = 0;
    int end_idx = 0;
    switch (type) {
        case MarkerType::BCE_WALL:
            start_idx = (int)m_data_mgr.countersH->startBoundaryMarkers;
            end_idx = start_idx + (int)m_data_mgr.countersH->numBoundaryMarkers;
            break;
        case MarkerType::SPH_PARTICLE:
            start_idx = 0;
            end_idx = start_idx + (int)m_data_mgr.countersH->numFluidMarkers;
            break;
    }

    // Transform all markers in the specified range
    thrust::for_each(m_data_mgr.sphMarkers_D->iterator(start_idx), m_data_mgr.sphMarkers_D->iterator(end_idx),
                     shift_op(shift, m_props));
}

// Selector function to find particles in a given AABB.
// Implements a Thrust predicate to be used with thrust::transform_if or thrust::partition.
struct inaabb_op {
    inaabb_op(const RealAABB& aabb_src) : aabb(aabb_src) {}

    template <typename T>
    __device__ bool operator()(const T& a) const {
        Real4 posw = thrust::get<0>(a);
        Real3 pos = mR3(posw);
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

// Relocation function to move particles to a given integer AABB.
// Implements a Thrust unary function to be used with thrust::for_each.
// Operates on a tuple {index, data_tuple}.
struct togrid_op {
    togrid_op(const IntAABB& aabb_dest, Real spacing, const SphParticleRelocator::DefaultProperties& props)
        : aabb(aabb_dest), delta(spacing), p(props) {}

    template <typename T>
    __device__ T operator()(const T& t) const {
        int index = thrust::get<0>(t);
        auto a = thrust::get<1>(t);

        // 1. Convert linear index to 3D grid coordinates in an AABB of same size as destination AABB
        int idx = index;
        auto dim = aabb.max - aabb.min;
        int x = idx % (dim.x + 1);
        idx /= (dim.x + 1);
        int y = idx % (dim.y + 1);
        idx /= (dim.y + 1);
        int z = idx;

        // 2. Shift marker grid ccordinates to current destination AABB
        x += aabb.min.x;
        y += aabb.min.y;
        z += aabb.min.z;

        // Modify marker position in real coordinates (preserve marker type)
        auto w = thrust::get<0>(a).w;
        Real3 pos = mR3(delta * x, delta * y, delta * z);
        thrust::get<0>(a) = mR4(pos, w);

        // Reset all other marker properties
        Real3 zero = mR3(0);
        thrust::get<1>(a) = zero;                                        // velocity
        thrust::get<2>(a) = mR4(p.rho0, 0, p.mu0, thrust::get<2>(a).w);  // rho, pres, mu, type
        thrust::get<3>(a) = zero;                                        // tau diagonal
        thrust::get<4>(a) = zero;                                        // tau off-diagonal

        return t;
    }

    IntAABB aabb;
    Real delta;
    SphParticleRelocator::DefaultProperties p;
};

void SphParticleRelocator::MoveAABB2AABB(MarkerType type,
                                         const RealAABB& aabb_src,
                                         const IntAABB& aabb_dest,
                                         Real spacing) {
    // Get start and end indices in marker data vectors based on specified type
    int start_idx = 0;
    int end_idx = 0;
    switch (type) {
        case MarkerType::BCE_WALL:
            start_idx = (int)m_data_mgr.countersH->startBoundaryMarkers;
            end_idx = start_idx + (int)m_data_mgr.countersH->numBoundaryMarkers;
            break;
        case MarkerType::SPH_PARTICLE:
            start_idx = 0;
            end_idx = start_idx + (int)m_data_mgr.countersH->numFluidMarkers;
            break;
    }

    // Move markers to be relocated at beginning of data structure
    auto middle = thrust::partition(m_data_mgr.sphMarkers_D->iterator(start_idx),
                                    m_data_mgr.sphMarkers_D->iterator(end_idx), inaabb_op(aabb_src));

    auto n_move = (int)(middle - m_data_mgr.sphMarkers_D->iterator(start_idx));

    ChDebugLog("Num candidate markers: " << m_data_mgr.sphMarkers_D->iterator(end_idx) -
                                                m_data_mgr.sphMarkers_D->iterator(start_idx));
    ChDebugLog("Num moved markers:     " << n_move);

    // Relocate markers based on their index
    thrust::counting_iterator<uint> idx_first(0);
    thrust::counting_iterator<uint> idx_last = idx_first + n_move;

    auto data_first = m_data_mgr.sphMarkers_D->iterator(start_idx);
    auto data_last = m_data_mgr.sphMarkers_D->iterator(start_idx + n_move);

    thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(idx_first, data_first)),
                     thrust::make_zip_iterator(thrust::make_tuple(idx_last, data_last)),
                     togrid_op(aabb_dest, spacing, m_props));
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
