// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#include <algorithm>
#include <climits>

#include "chrono/collision/multicore/ChBroadphase.h"
#include "chrono/collision/multicore/ChCollisionUtils.h"

// Always include ChConfig.h *before* any Thrust headers!
#include "chrono/ChConfig.h"
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>
#include <thrust/sort.h>
#include <thrust/sequence.h>
#include <thrust/iterator/constant_iterator.h>

using thrust::transform;
using thrust::transform_reduce;

namespace chrono {

using namespace chrono::ch_utils;

ChBroadphase::ChBroadphase()
    : grid_type(GridType::FIXED_RESOLUTION),
      grid_resolution(vec3(10, 10, 10)),
      bin_size(real3(1, 1, 1)),
      grid_density(5),
      cd_data(nullptr) {}

// -----------------------------------------------------------------------------

// Inverted AABB (assumed associated with an active shape).
auto inverted =
    thrust::make_tuple(real3(+C_REAL_MAX, +C_REAL_MAX, +C_REAL_MAX), real3(-C_REAL_MAX, -C_REAL_MAX, -C_REAL_MAX), 0);

// Invert an AABB associated with an inactive shape or a shape on a non-colliding body.
struct BoxInvert {
    BoxInvert(const std::vector<char>* collide) : m_collide(collide) {}
    thrust::tuple<real3, real3, uint> operator()(const thrust::tuple<real3, real3, uint>& lhs) {
        uint lhs_id = thrust::get<2>(lhs);
        if (lhs_id == UINT_MAX || (*m_collide)[lhs_id] == 0)
            return inverted;
        else
            return lhs;
    }
    const std::vector<char>* m_collide;
};

// AABB union reduction operator
struct BoxReduce {
    thrust::tuple<real3, real3, uint> operator()(const thrust::tuple<real3, real3, uint>& lhs,
                                                 const thrust::tuple<real3, real3, uint>& rhs) {
        real3 lhs_ll = thrust::get<0>(lhs);
        real3 lhs_ur = thrust::get<1>(lhs);

        real3 rhs_ll = thrust::get<0>(rhs);
        real3 rhs_ur = thrust::get<1>(rhs);

        real3 ll = real3(Min(lhs_ll.x, rhs_ll.x), Min(lhs_ll.y, rhs_ll.y), Min(lhs_ll.z, rhs_ll.z));
        real3 ur = real3(Max(lhs_ur.x, rhs_ur.x), Max(lhs_ur.y, rhs_ur.y), Max(lhs_ur.z, rhs_ur.z));

        return thrust::tuple<real3, real3, uint>(ll, ur, 0);
    }
};

// Calculate AABB of all rigid shapes.
// This function excludes inactive shapes (marked with ID = UINT_MAX) and shapes
// associated with non-colliding bodies.
void ChBroadphase::RigidBoundingBox() {
    // Vectors of length = number of collision shapes
    const std::vector<real3>& aabb_min = cd_data->aabb_min;
    const std::vector<real3>& aabb_max = cd_data->aabb_max;
    const std::vector<uint>& id_rigid = cd_data->shape_data.id_rigid;

    // Vectors of length = number of rigid bodies
    const std::vector<char>& collide_rigid = *cd_data->state_data.collide_rigid;

    // Calculate union of all AABBs.
    // Excluded AABBs are inverted through the transform operation, prior to the reduction.
    auto begin = thrust::make_zip_iterator(thrust::make_tuple(aabb_min.begin(), aabb_max.begin(), id_rigid.begin()));
    auto end = thrust::make_zip_iterator(thrust::make_tuple(aabb_min.end(), aabb_max.end(), id_rigid.end()));
    auto result = thrust::transform_reduce(THRUST_PAR begin, end, BoxInvert(&collide_rigid), inverted, BoxReduce());

    cd_data->rigid_min_bounding_point = thrust::get<0>(result);
    cd_data->rigid_max_bounding_point = thrust::get<1>(result);
}

// AABB as the pair of its min and max corners.
typedef thrust::pair<real3, real3> bbox;

// Reduce a pair of bounding boxes (a,b) to a bounding box containing a and b.
struct bbox_reduction : public thrust::binary_function<bbox, bbox, bbox> {
    bbox operator()(bbox a, bbox b) {
        real3 ll = real3(Min(a.first.x, b.first.x), Min(a.first.y, b.first.y),
                         Min(a.first.z, b.first.z));  // lower left corner
        real3 ur = real3(Max(a.second.x, b.second.x), Max(a.second.y, b.second.y),
                         Max(a.second.z, b.second.z));  // upper right corner
        return bbox(ll, ur);
    }
};

// Convert a point to a bbox containing that point, (point) -> (point, point).
struct bbox_transformation : public thrust::unary_function<real3, bbox> {
    bbox operator()(real3 point) { return bbox(point, point); }
};

// Calculate AABB of all fluid particles.
void ChBroadphase::FluidBoundingBox() {
    const std::vector<real3>& pos_fluid = *cd_data->state_data.pos_3dof;
    const real radius = cd_data->p_kernel_radius + cd_data->p_collision_envelope;

    bbox res(pos_fluid[0], pos_fluid[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_fluid.begin(), pos_fluid.end(), unary_op, res, binary_op);

    res.first.x = radius * Floor(res.first.x / radius);
    res.first.y = radius * Floor(res.first.y / radius);
    res.first.z = radius * Floor(res.first.z / radius);

    res.second.x = radius * Ceil(res.second.x / radius);
    res.second.y = radius * Ceil(res.second.y / radius);
    res.second.z = radius * Ceil(res.second.z / radius);

    cd_data->ff_min_bounding_point = res.first - radius * 6;
    cd_data->ff_max_bounding_point = res.second + radius * 6;
}

void ChBroadphase::DetermineBoundingBox() {
    RigidBoundingBox();

    real3 min_point = cd_data->rigid_min_bounding_point;
    real3 max_point = cd_data->rigid_max_bounding_point;

    if (cd_data->state_data.num_fluid_bodies != 0) {
        FluidBoundingBox();
        min_point = Min(min_point, cd_data->ff_min_bounding_point);
        max_point = Max(max_point, cd_data->ff_max_bounding_point);
    }

    // Inflate the overall bounding box by a small percentage.
    // This takes care of corner cases where a degenerate object bounding box is on the
    // boundary of the overall bounding box.
    real fraction = 1e-3;
    real3 size = max_point - min_point;
    min_point = min_point - fraction * size;
    max_point = max_point + fraction * size;

    cd_data->min_bounding_point = min_point;
    cd_data->max_bounding_point = max_point;
    cd_data->global_origin = min_point;
}

void ChBroadphase::OffsetAABB() {
    std::vector<real3>& aabb_min = cd_data->aabb_min;
    std::vector<real3>& aabb_max = cd_data->aabb_max;

    thrust::constant_iterator<real3> offset(cd_data->global_origin);

    thrust::transform(aabb_min.begin(), aabb_min.end(), offset, aabb_min.begin(), thrust::minus<real3>());
    thrust::transform(aabb_max.begin(), aabb_max.end(), offset, aabb_max.begin(), thrust::minus<real3>());
}

// Determine resolution of the top level grid
void ChBroadphase::ComputeTopLevelResolution() {
    const int num_shapes = cd_data->num_rigid_shapes;
    const real3& max_bounding_point = cd_data->max_bounding_point;
    const real3& global_origin = cd_data->global_origin;

    vec3& bins_per_axis = cd_data->bins_per_axis;

    // This is the extents of the space (overall grid diagonal)
    real3 diag = (Abs(max_bounding_point - global_origin));

    // Compute number of bins (grid resolution)
    switch (grid_type) {
        case GridType::FIXED_RESOLUTION:
            bins_per_axis = grid_resolution;
            break;
        case GridType::FIXED_BIN_SIZE:
            bins_per_axis.x = (int)std::ceil(diag.x / bin_size.x);
            bins_per_axis.y = (int)std::ceil(diag.y / bin_size.y);
            bins_per_axis.z = (int)std::ceil(diag.z / bin_size.z);
            break;
        case GridType::FIXED_DENSITY:
            bins_per_axis = Compute_Grid_Resolution(num_shapes, diag, grid_density);
    }

    // Calculate actual bin dimension
    cd_data->bin_size = diag / real3(bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);

    // Cache the reciprocal bin size
    cd_data->inv_bin_size = 1.0 / cd_data->bin_size;
}

// -----------------------------------------------------------------------------

// Use spatial subdivision to detect the list of POSSIBLE collisions
void ChBroadphase::Process() {
    // Compute overall AABB and then offset all AABBs
    DetermineBoundingBox();
    OffsetAABB();

    // Determine resolution of the top level grid
    ComputeTopLevelResolution();

    if (cd_data->num_rigid_shapes != 0) {
        OneLevelBroadphase();
        cd_data->num_rigid_contacts = cd_data->num_possible_collisions;
    }
    return;
}

void ChBroadphase::OneLevelBroadphase() {
    const std::vector<uint>& obj_data_id = cd_data->shape_data.id_rigid;
    const std::vector<short2>& fam_data = cd_data->shape_data.fam_rigid;

    const std::vector<char>& obj_active = *cd_data->state_data.active_rigid;
    const std::vector<char>& obj_collide = *cd_data->state_data.collide_rigid;

    const std::vector<real3>& aabb_min = cd_data->aabb_min;
    const std::vector<real3>& aabb_max = cd_data->aabb_max;
    std::vector<long long>& pair_shapeIDs = cd_data->pair_shapeIDs;
    std::vector<uint>& bin_intersections = cd_data->bin_intersections;
    std::vector<uint>& bin_number = cd_data->bin_number;
    std::vector<uint>& bin_aabb_number = cd_data->bin_aabb_number;
    std::vector<uint>& bin_active = cd_data->bin_active;
    std::vector<uint>& bin_start_index = cd_data->bin_start_index;
    std::vector<uint>& bin_start_index_ext = cd_data->bin_start_index_ext;
    std::vector<uint>& bin_num_contact = cd_data->bin_num_contact;

    const int num_shapes = cd_data->num_rigid_shapes;

    const vec3& bins_per_axis = cd_data->bins_per_axis;
    const real3& inv_bin_size = cd_data->inv_bin_size;
    uint& num_bins = cd_data->num_bins;
    uint& num_active_bins = cd_data->num_active_bins;
    uint& num_bin_aabb_intersections = cd_data->num_bin_aabb_intersections;
    uint& num_possible_collisions = cd_data->num_possible_collisions;

    num_bins = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    bin_intersections.resize(num_shapes + 1);
    bin_intersections[num_shapes] = 0;

    // Count the number of bins intersected by each shape AABB -> bin_intersections
#pragma omp parallel for
    for (int i = 0; i < num_shapes; i++) {
        if (obj_data_id[i] == UINT_MAX) {
            bin_intersections[i] = 0;
            continue;
        }
        f_Count_AABB_BIN_Intersection(i, inv_bin_size, aabb_min, aabb_max, bin_intersections);
    }

    // Calculate total number of bin - shape AABB intersections
    Thrust_Exclusive_Scan(bin_intersections);
    num_bin_aabb_intersections = bin_intersections.back();

    bin_number.resize(num_bin_aabb_intersections);
    bin_aabb_number.resize(num_bin_aabb_intersections);
    bin_active.resize(num_bin_aabb_intersections);       // will be resized after calculation of num_active_bins
    bin_start_index.resize(num_bin_aabb_intersections);  // will be resized after calculation of num_active_bins

    // For each shape, store the bin index and the shape ID for intersections with this shape
#pragma omp parallel for
    for (int i = 0; i < num_shapes; i++) {
        if (obj_data_id[i] == UINT_MAX)
            continue;
        f_Store_AABB_BIN_Intersection(i, bins_per_axis, inv_bin_size, aabb_min, aabb_max, bin_intersections, bin_number,
                                      bin_aabb_number);
    }

    // Find the number of active bins (i.e. with at least one shape AABB intersection)
    Thrust_Sort_By_Key(bin_number, bin_aabb_number);
    num_active_bins = (int)(Run_Length_Encode(bin_number, bin_active, bin_start_index));

    if (num_active_bins <= 0) {
        num_possible_collisions = 0;
        return;
    }

    bin_active.resize(num_active_bins);
    bin_start_index.resize(num_active_bins + 1);
    bin_start_index[num_active_bins] = 0;

    Thrust_Exclusive_Scan(bin_start_index);
    bin_num_contact.resize(num_active_bins + 1);
    bin_num_contact[num_active_bins] = 0;

    // Count the number of AABB-AABB intersections in each active bin -> bin_num_contact
#pragma omp parallel for
    for (int i = 0; i < (signed)num_active_bins; i++) {
        f_Count_AABB_AABB_Intersection(i, inv_bin_size, bins_per_axis, aabb_min, aabb_max, bin_active, bin_aabb_number,
                                       bin_start_index, fam_data, obj_active, obj_collide, obj_data_id,
                                       bin_num_contact);
    }

    thrust::exclusive_scan(bin_num_contact.begin(), bin_num_contact.end(), bin_num_contact.begin());
    num_possible_collisions = bin_num_contact.back();
    pair_shapeIDs.resize(num_possible_collisions);

    // Store the list of shape pairs in potential collision (i.e. with intersecting AABBs)
#pragma omp parallel for
    for (int index = 0; index < (signed)num_active_bins; index++) {
        f_Store_AABB_AABB_Intersection(index, inv_bin_size, bins_per_axis, aabb_min, aabb_max, bin_active,
                                       bin_aabb_number, bin_start_index, bin_num_contact, fam_data, obj_active,
                                       obj_collide, obj_data_id, pair_shapeIDs);
    }

    pair_shapeIDs.resize(num_possible_collisions);

    // For use in ray intersection tests, also create an "extended" vector of start indices that also includes bins with
    // no shape AABB intersections.
    bin_start_index_ext.resize(num_bins + 1);

#pragma omp parallel for
    for (int j = 0; j <= (signed)bin_active[0]; j++) {
        bin_start_index_ext[j] = bin_start_index[0];
    }
#pragma omp parallel for
    for (int index = 1; index < (signed)num_active_bins; index++) {
        // Set the extended array for the current active bin as well as any empty bins before it.
        for (uint j = bin_active[index - 1] + 1; j <= bin_active[index]; j++)
            bin_start_index_ext[j] = bin_start_index[index];
    }
#pragma omp parallel for
    for (int j = bin_active[num_active_bins - 1] + 1; j <= (signed)num_bins; j++) {
        bin_start_index_ext[j] = bin_start_index[num_active_bins];
    }
}

}  // end namespace chrono
