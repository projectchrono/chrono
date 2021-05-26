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

#include "chrono/collision/chrono/ChBroadphase.h"
#include "chrono/collision/chrono/ChBroadphaseUtils.h"

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
namespace collision {

ChBroadphase::ChBroadphase() : bins_per_axis(vec3(10, 10, 10)), fixed_bins(true), grid_density(5), cd_data(nullptr) {}

// Determine the bounding box for the objects===============================================================

// Inverted AABB (assumed associated with an active shape).
auto inverted = thrust::make_tuple(real3(+C_LARGE_REAL, +C_LARGE_REAL, +C_LARGE_REAL),
                                   real3(-C_LARGE_REAL, -C_LARGE_REAL, -C_LARGE_REAL),
                                   0);

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
    const std::vector<real3>& aabb_min = cd_data->host_data.aabb_min;
    const std::vector<real3>& aabb_max = cd_data->host_data.aabb_max;
    const std::vector<uint>& id_rigid = cd_data->shape_data.id_rigid;

    // Vectors of length = number of rigid bodies
    const std::vector<char>& collide_rigid = *cd_data->state_data.collide_rigid;

    // Calculate union of all AABBs.
    // Excluded AABBs are inverted through the transform operation, prior to the reduction.
    auto begin = thrust::make_zip_iterator(thrust::make_tuple(aabb_min.begin(), aabb_max.begin(), id_rigid.begin()));
    auto end = thrust::make_zip_iterator(thrust::make_tuple(aabb_min.end(), aabb_max.end(), id_rigid.end()));
    auto result = thrust::transform_reduce(THRUST_PAR begin, end, BoxInvert(&collide_rigid), inverted, BoxReduce());

    cd_data->measures.rigid_min_bounding_point = thrust::get<0>(result);
    cd_data->measures.rigid_max_bounding_point = thrust::get<1>(result);
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
    const real radius = cd_data->node_data.kernel_radius + cd_data->node_data.collision_envelope;

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

    real3& min_bounding_point = cd_data->measures.ff_min_bounding_point;
    real3& max_bounding_point = cd_data->measures.ff_max_bounding_point;

    max_bounding_point = res.second + radius * 6;
    min_bounding_point = res.first - radius * 6;
}

void ChBroadphase::DetermineBoundingBox() {
    RigidBoundingBox();

    real3 min_point = cd_data->measures.rigid_min_bounding_point;
    real3 max_point = cd_data->measures.rigid_max_bounding_point;

    if (cd_data->state_data.num_fluid_bodies != 0) {
        FluidBoundingBox();
        min_point = Min(min_point, cd_data->measures.ff_min_bounding_point);
        max_point = Max(max_point, cd_data->measures.ff_max_bounding_point);
    }

    // Inflate the overall bounding box by a small percentage.
    // This takes care of corner cases where a degenerate object bounding box is on the
    // boundary of the overall bounding box.
    real fraction = 1e-3;
    real3 size = max_point - min_point;
    min_point = min_point - fraction * size;
    max_point = max_point + fraction * size;

    cd_data->measures.min_bounding_point = min_point;
    cd_data->measures.max_bounding_point = max_point;
    cd_data->measures.global_origin = min_point;
}

void ChBroadphase::OffsetAABB() {
    std::vector<real3>& aabb_min = cd_data->host_data.aabb_min;
    std::vector<real3>& aabb_max = cd_data->host_data.aabb_max;

    thrust::constant_iterator<real3> offset(cd_data->measures.global_origin);

    thrust::transform(aabb_min.begin(), aabb_min.end(), offset, aabb_min.begin(), thrust::minus<real3>());
    thrust::transform(aabb_max.begin(), aabb_max.end(), offset, aabb_max.begin(), thrust::minus<real3>());
}

// Determine resolution of the top level grid
void ChBroadphase::ComputeTopLevelResolution() {
    const int num_shapes = cd_data->num_rigid_shapes;
    const real3& max_bounding_point = cd_data->measures.max_bounding_point;
    const real3& global_origin = cd_data->measures.global_origin;

    real3& inv_bin_size = cd_data->measures.inv_bin_size;
    real3& bin_size = cd_data->measures.bin_size;

    // This is the extents of the space aka diameter
    real3 diagonal = (Abs(max_bounding_point - global_origin));

    // Compute the number of slices in this grid level
    if (!fixed_bins) {
        bins_per_axis = function_Compute_Grid_Resolution(num_shapes, diagonal, grid_density);
    }
    bin_size = diagonal / real3(bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);

    // Store the inverse for use later
    inv_bin_size = 1.0 / bin_size;
}

// =========================================================================================================

// use spatial subdivision to detect the list of POSSIBLE collisions
// let user define their own narrow-phase collision detection
void ChBroadphase::DispatchRigid() {
    if (cd_data->num_rigid_shapes != 0) {
        OneLevelBroadphase();
        cd_data->num_rigid_contacts = cd_data->measures.number_of_contacts_possible;
    }
    return;
}

void ChBroadphase::OneLevelBroadphase() {
    const std::vector<uint>& obj_data_id = cd_data->shape_data.id_rigid;
    const std::vector<short2>& fam_data = cd_data->shape_data.fam_rigid;

    const std::vector<char>& obj_active = *cd_data->state_data.active_rigid;
    const std::vector<char>& obj_collide = *cd_data->state_data.collide_rigid;

    const std::vector<real3>& aabb_min = cd_data->host_data.aabb_min;
    const std::vector<real3>& aabb_max = cd_data->host_data.aabb_max;
    std::vector<long long>& pair_shapeIDs = cd_data->host_data.pair_shapeIDs;
    std::vector<uint>& bin_intersections = cd_data->host_data.bin_intersections;
    std::vector<uint>& bin_number = cd_data->host_data.bin_number;
    std::vector<uint>& bin_number_out = cd_data->host_data.bin_number_out;
    std::vector<uint>& bin_aabb_number = cd_data->host_data.bin_aabb_number;
    std::vector<uint>& bin_start_index = cd_data->host_data.bin_start_index;
    std::vector<uint>& bin_num_contact = cd_data->host_data.bin_num_contact;

    const int num_shapes = cd_data->num_rigid_shapes;

    real3& inv_bin_size = cd_data->measures.inv_bin_size;
    uint& number_of_bins_active = cd_data->measures.number_of_bins_active;
    uint& number_of_bin_intersections = cd_data->measures.number_of_bin_intersections;
    uint& number_of_contacts_possible = cd_data->measures.number_of_contacts_possible;

    bin_intersections.resize(num_shapes + 1);
    bin_intersections[num_shapes] = 0;

#pragma omp parallel for
    for (int i = 0; i < num_shapes; i++) {
        if (obj_data_id[i] == UINT_MAX) {
            bin_intersections[i] = 0;
            continue;
        }
        f_Count_AABB_BIN_Intersection(i, inv_bin_size, aabb_min, aabb_max, bin_intersections);
    }

    Thrust_Exclusive_Scan(bin_intersections);
    number_of_bin_intersections = bin_intersections.back();

    bin_number.resize(number_of_bin_intersections);
    bin_number_out.resize(number_of_bin_intersections);
    bin_aabb_number.resize(number_of_bin_intersections);
    bin_start_index.resize(number_of_bin_intersections);

#pragma omp parallel for
    for (int i = 0; i < num_shapes; i++) {
        if (obj_data_id[i] == UINT_MAX)
            continue;
        f_Store_AABB_BIN_Intersection(i, bins_per_axis, inv_bin_size, aabb_min, aabb_max, bin_intersections, bin_number,
                                      bin_aabb_number);
    }

    Thrust_Sort_By_Key(bin_number, bin_aabb_number);
    number_of_bins_active = (int)(Run_Length_Encode(bin_number, bin_number_out, bin_start_index));

    if (number_of_bins_active <= 0) {
        number_of_contacts_possible = 0;
        return;
    }

    bin_start_index.resize(number_of_bins_active + 1);
    bin_start_index[number_of_bins_active] = 0;

    Thrust_Exclusive_Scan(bin_start_index);
    bin_num_contact.resize(number_of_bins_active + 1);
    bin_num_contact[number_of_bins_active] = 0;

#pragma omp parallel for
    for (int i = 0; i < (signed)number_of_bins_active; i++) {
        f_Count_AABB_AABB_Intersection(i, inv_bin_size, bins_per_axis, aabb_min, aabb_max, bin_number_out,
                                       bin_aabb_number, bin_start_index, fam_data, obj_active, obj_collide, obj_data_id,
                                       bin_num_contact);
    }

    thrust::exclusive_scan(bin_num_contact.begin(), bin_num_contact.end(), bin_num_contact.begin());
    number_of_contacts_possible = bin_num_contact.back();
    pair_shapeIDs.resize(number_of_contacts_possible);

#pragma omp parallel for
    for (int index = 0; index < (signed)number_of_bins_active; index++) {
        f_Store_AABB_AABB_Intersection(index, inv_bin_size, bins_per_axis, aabb_min, aabb_max, bin_number_out,
                                       bin_aabb_number, bin_start_index, bin_num_contact, fam_data, obj_active,
                                       obj_collide, obj_data_id, pair_shapeIDs);
    }

    pair_shapeIDs.resize(number_of_contacts_possible);
}

}  // end namespace collision
}  // end namespace chrono
