// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Utility functions used by multiple parts of the collision code
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChDataManager.h"

#include <thrust/pair.h>
#include <thrust/functional.h>

namespace chrono {
namespace collision {

typedef thrust::pair<real3, real3> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction : public thrust::binary_function<bbox, bbox, bbox> {
    bbox operator()(bbox a, bbox b) {
        real3 ll = real3(Min(a.first.x, b.first.x), Min(a.first.y, b.first.y),
                         Min(a.first.z, b.first.z));  // lower left corner
        real3 ur = real3(Max(a.second.x, b.second.x), Max(a.second.y, b.second.y),
                         Max(a.second.z, b.second.z));  // upper right corner
        return bbox(ll, ur);
    }
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation : public thrust::unary_function<real3, bbox> {
    bbox operator()(real3 point) { return bbox(point, point); }
};

// HASHING FUNCTIONS =======================================================================================
// Convert a position into a bin index
template <class T>
inline vec3 HashMin(const T& A, const real3& inv_bin_size_vec) {
    vec3 temp;
    temp.x = (int)Floor(A.x * inv_bin_size_vec.x);
    temp.y = (int)Floor(A.y * inv_bin_size_vec.y);
    temp.z = (int)Floor(A.z * inv_bin_size_vec.z);
    return temp;
}

template <class T>
inline vec3 HashMax(const T& A, const real3& inv_bin_size_vec) {
    vec3 temp;
    temp.x = (int)Ceil(A.x * inv_bin_size_vec.x) - 1;
    temp.y = (int)Ceil(A.y * inv_bin_size_vec.y) - 1;
    temp.z = (int)Ceil(A.z * inv_bin_size_vec.z) - 1;
    return temp;
}

// Convert a bin index into a unique hash value
static inline uint Hash_Index(const vec3& A, vec3 bins_per_axis) {
    return ((A.z * bins_per_axis.y) * bins_per_axis.x) + (A.y * bins_per_axis.x) + A.x;
}
// Decodes a hash into it's associated bin position
static inline vec3 Hash_Decode(uint hash, vec3 bins_per_axis) {
    vec3 decoded_hash;
    decoded_hash.x = hash % (bins_per_axis.x * bins_per_axis.y) % bins_per_axis.x;
    decoded_hash.y = (hash % (bins_per_axis.x * bins_per_axis.y)) / bins_per_axis.x;
    decoded_hash.z = hash / (bins_per_axis.x * bins_per_axis.y);
    return decoded_hash;
}

// AABB COLLISION FUNCTIONS ================================================================================

// Check if two bodies interact using their collision family data.
static inline bool collide(short2 fam_data_A, short2 fam_data_B) {
    // Return true only if the bit corresponding to family of B is set in the mask
    // of A and vice-versa.
    return (fam_data_A.y & fam_data_B.x) && (fam_data_B.y & fam_data_A.x);
}

// Check if two AABBs overlap using their min/max corners.
static inline bool overlap(real3 Amin, real3 Amax, real3 Bmin, real3 Bmax) {
    // Return true only if the two AABBs overlap in all 3 directions.
    return (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
           (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
}

static inline bool
current_bin(real3 Amin, real3 Amax, real3 Bmin, real3 Bmax, real3 inv_bin_size_vec, vec3 bins_per_axis, uint bin) {
    real3 min_p = Max(Amin, Bmin);
    real3 max_p = Min(Amax, Bmax);

    real3 center = (min_p + max_p) * 0.5;  // center of contact volume

    if (Hash_Index(HashMin(min_p, inv_bin_size_vec), bins_per_axis) == bin) {
        return true;
    }
    return false;
}

// Grid Size FUNCTIONS =====================================================================================

// for a given number of aabbs in a grid, the grids maximum and minimum point along with the density factor
// compute the size of the grid
static vec3 function_Compute_Grid_Resolution(uint num_aabb, real3 d, real k = .1) {
    vec3 grid_size = vec3(0);
    real V = d.x * d.y * d.z;
    grid_size.x = int(d.x * Pow(k * num_aabb / V, real(1.0 / 3.0)));
    grid_size.y = int(d.y * Pow(k * num_aabb / V, real(1.0 / 3.0)));
    grid_size.z = int(d.z * Pow(k * num_aabb / V, real(1.0 / 3.0)));

    grid_size = Clamp(grid_size, vec3(1), grid_size);

    return grid_size;
}

// =========================================================================================================

static bool function_Check_Sphere(real3 pos_a, real3 pos_b, real radius) {
    real3 delta = pos_b - pos_a;
    real dist2 = Dot(delta, delta);
    real radSum = radius + radius;
    if (dist2 >= radSum * radSum || dist2 < 1e-12) {
        return false;
    }
    return true;
}
// TWO LEVEL FUNCTIONS==========================================================

// For each bin determine the grid size and store it========================================================
static void f_TL_Count_Leaves(const uint index,
                              const real density,
                              const real3& bin_size,
                              const custom_vector<uint>& bin_start_index,
                              custom_vector<uint>& leaves_per_bin) {
    uint start = bin_start_index[index];
    uint end = bin_start_index[index + 1];
    uint num_aabb_in_cell = end - start;

    vec3 cell_res = function_Compute_Grid_Resolution(num_aabb_in_cell, bin_size, density);

    leaves_per_bin[index] = cell_res.x * cell_res.y * cell_res.z;
}
// Count the number of AABB leaf intersections for each bin=================================================
static void f_TL_Count_AABB_Leaf_Intersection(const uint index,
                                              const real density,
                                              const real3& bin_size,
                                              const vec3& bins_per_axis,
                                              const custom_vector<uint>& bin_start_index,
                                              const custom_vector<uint>& bin_number,
                                              const custom_vector<uint>& shape_number,
                                              const custom_vector<real3>& aabb_min,
                                              const custom_vector<real3>& aabb_max,
                                              custom_vector<uint>& leaves_intersected) {
    uint start = bin_start_index[index];
    uint end = bin_start_index[index + 1];
    uint count = 0;
    uint num_aabb_in_cell = end - start;
    vec3 cell_res = function_Compute_Grid_Resolution(num_aabb_in_cell, bin_size, density);
    real3 inv_leaf_size = real3(cell_res.x, cell_res.y, cell_res.z) / bin_size;
    vec3 bin_index = Hash_Decode(bin_number[index], bins_per_axis);
    real3 bin_position = real3(bin_index.x * bin_size.x, bin_index.y * bin_size.y, bin_index.z * bin_size.z);

    for (uint i = start; i < end; i++) {
        uint shape = shape_number[i];
        // subtract the bin position from the AABB position
        real3 Amin = aabb_min[shape] - bin_position;
        real3 Amax = aabb_max[shape] - bin_position;

        // Make sure that even with subtraction we are at the origin
        Amin = Clamp(Amin, real3(0), Amax);

        // Find the extents
        vec3 gmin = HashMin(Amin, inv_leaf_size);
        vec3 gmax = HashMax(Amax, inv_leaf_size);
        // Make sure that the maximum bin value does not exceed the bounds of this grid
        vec3 max_clamp = cell_res - vec3(1);
        gmin = Clamp(gmin, vec3(0), max_clamp);
        gmax = Clamp(gmax, vec3(0), max_clamp);

        count += (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }

    leaves_intersected[index] = count;
}
// Store the AABB leaf intersections for each bin===========================================================
static void f_TL_Write_AABB_Leaf_Intersection(const uint& index,
                                              const real density,
                                              const real3& bin_size,
                                              const vec3& bin_resolution,
                                              const custom_vector<uint>& bin_start_index,
                                              const custom_vector<uint>& bin_number,
                                              const custom_vector<uint>& bin_shape_number,
                                              const custom_vector<real3>& aabb_min,
                                              const custom_vector<real3>& aabb_max,
                                              const custom_vector<uint>& leaves_intersected,
                                              const custom_vector<uint>& leaves_per_bin,
                                              custom_vector<uint>& leaf_number,
                                              custom_vector<uint>& leaf_shape_number) {
    uint start = bin_start_index[index];
    uint end = bin_start_index[index + 1];
    uint mInd = leaves_intersected[index];
    uint count = 0;
    uint num_aabb_in_cell = end - start;
    vec3 cell_res = function_Compute_Grid_Resolution(num_aabb_in_cell, bin_size, density);
    real3 inv_leaf_size = real3(cell_res.x, cell_res.y, cell_res.z) / bin_size;

    vec3 bin_index = Hash_Decode(bin_number[index], bin_resolution);

    real3 bin_position = real3(bin_index.x * bin_size.x, bin_index.y * bin_size.y, bin_index.z * bin_size.z);

    for (uint i = start; i < end; i++) {
        uint shape = bin_shape_number[i];
        // subtract the bin position from the AABB position
        real3 Amin = aabb_min[shape] - bin_position;
        real3 Amax = aabb_max[shape] - bin_position;

        // Make sure that even with subtraction we are at the origin
        Amin = Clamp(Amin, real3(0), Amax);

        // Find the extents
        vec3 gmin = HashMin(Amin, inv_leaf_size);
        vec3 gmax = HashMax(Amax, inv_leaf_size);

        // Make sure that the maximum bin value does not exceed the bounds of this grid
        vec3 max_clamp = cell_res - vec3(1);
        gmin = Clamp(gmin, vec3(0), max_clamp);
        gmax = Clamp(gmax, vec3(0), max_clamp);

        int a, b, c;
        for (a = gmin.x; a <= gmax.x; a++) {
            for (b = gmin.y; b <= gmax.y; b++) {
                for (c = gmin.z; c <= gmax.z; c++) {
                    leaf_number[mInd + count] = leaves_per_bin[index] + Hash_Index(vec3(a, b, c), cell_res);
                    leaf_shape_number[mInd + count] = shape;
                    count++;
                }
            }
        }
    }
}

// ONE AND TWO LEVEL FUNCTIONS==========================================================

// Function to Count AABB Bin intersections=================================================================
static inline void f_Count_AABB_BIN_Intersection(const uint index,
                                                 const real3& inv_bin_size,
                                                 const custom_vector<real3>& aabb_min,
                                                 const custom_vector<real3>& aabb_max,
                                                 custom_vector<uint>& bins_intersected) {
    vec3 gmin = HashMin(aabb_min[index], inv_bin_size);
    vec3 gmax = HashMax(aabb_max[index], inv_bin_size);
    bins_intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
}

// Function to Store AABB Bin Intersections=================================================================
static inline void f_Store_AABB_BIN_Intersection(const uint index,
                                                 const vec3& bins_per_axis,
                                                 const real3& inv_bin_size,
                                                 const custom_vector<real3>& aabb_min_data,
                                                 const custom_vector<real3>& aabb_max_data,
                                                 const custom_vector<uint>& bins_intersected,
                                                 custom_vector<uint>& bin_number,
                                                 custom_vector<uint>& aabb_number) {
    uint count = 0, i, j, k;
    vec3 gmin = HashMin(aabb_min_data[index], inv_bin_size);
    vec3 gmax = HashMax(aabb_max_data[index], inv_bin_size);
    uint mInd = bins_intersected[index];
    for (i = gmin.x; i <= (uint)gmax.x; i++) {
        for (j = gmin.y; j <= (uint)gmax.y; j++) {
            for (k = gmin.z; k <= (uint)gmax.z; k++) {
                bin_number[mInd + count] = Hash_Index(vec3(i, j, k), bins_per_axis);
                aabb_number[mInd + count] = index;
                count++;
            }
        }
    }
}

// Function to count AABB AABB intersection=================================================================
static inline void f_Count_AABB_AABB_Intersection(const uint index,
                                                  const real3 inv_bin_size_vec,
                                                  const vec3 bins_per_axis,
                                                  const custom_vector<real3>& aabb_min_data,
                                                  const custom_vector<real3>& aabb_max_data,
                                                  const custom_vector<uint>& bin_number,
                                                  const custom_vector<uint>& aabb_number,
                                                  const custom_vector<uint>& bin_start_index,
                                                  const custom_vector<short2>& fam_data,
                                                  const custom_vector<char>& body_active,
                                                  const custom_vector<char>& body_collide,
                                                  const custom_vector<uint>& body_id,
                                                  custom_vector<uint>& num_contact) {
    uint start = bin_start_index[index];
    uint end = bin_start_index[index + 1];
    uint count = 0;
    // Terminate early if there is only one object in the bin
    if (end - start == 1) {
        num_contact[index] = 0;
        return;
    }
    for (uint i = start; i < end; i++) {
        uint shapeA = aabb_number[i];
        real3 Amin = aabb_min_data[shapeA];
        real3 Amax = aabb_max_data[shapeA];
        short2 famA = fam_data[shapeA];
        uint bodyA = body_id[shapeA];

        if (body_collide[bodyA] == 0)
            continue;

        for (uint k = i + 1; k < end; k++) {
            uint shapeB = aabb_number[k];
            uint bodyB = body_id[shapeB];
            real3 Bmin = aabb_min_data[shapeB];
            real3 Bmax = aabb_max_data[shapeB];

            if (shapeA == shapeB)
                continue;
            if (bodyA == bodyB)
                continue;
            if (body_collide[bodyB] == 0)
                continue;
            if (!body_active[bodyA] && !body_active[bodyB])
                continue;
            if (!collide(famA, fam_data[shapeB]))
                continue;
            if (!overlap(Amin, Amax, Bmin, Bmax))
                continue;
            if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size_vec, bins_per_axis, bin_number[index]) == false)
                continue;
            count++;
        }
    }

    num_contact[index] = count;
}

// Function to store AABB-AABB intersections================================================================
static inline void f_Store_AABB_AABB_Intersection(const uint index,
                                                  const real3 inv_bin_size_vec,
                                                  const vec3 bins_per_axis,
                                                  const custom_vector<real3>& aabb_min_data,
                                                  const custom_vector<real3>& aabb_max_data,
                                                  const custom_vector<uint>& bin_number,
                                                  const custom_vector<uint>& aabb_number,
                                                  const custom_vector<uint>& bin_start_index,
                                                  const custom_vector<uint>& num_contact,
                                                  const custom_vector<short2>& fam_data,
                                                  const custom_vector<char>& body_active,
                                                  const custom_vector<char>& body_collide,
                                                  const custom_vector<uint>& body_id,
                                                  custom_vector<long long>& potential_contacts) {
    uint start = bin_start_index[index];
    uint end = bin_start_index[index + 1];
    // Terminate early if there is only one object in the bin
    if (end - start == 1) {
        return;
    }
    uint offset = num_contact[index];
    uint count = 0;

    for (uint i = start; i < end; i++) {
        uint shapeA = aabb_number[i];
        real3 Amin = aabb_min_data[shapeA];
        real3 Amax = aabb_max_data[shapeA];
        short2 famA = fam_data[shapeA];
        uint bodyA = body_id[shapeA];

        if (body_collide[bodyA] == 0)
            continue;

        for (uint k = i + 1; k < end; k++) {
            uint shapeB = aabb_number[k];
            uint bodyB = body_id[shapeB];
            real3 Bmin = aabb_min_data[shapeB];
            real3 Bmax = aabb_max_data[shapeB];

            if (shapeA == shapeB)
                continue;
            if (bodyA == bodyB)
                continue;
            if (body_collide[bodyB] == 0)
                continue;
            if (!body_active[bodyA] && !body_active[bodyB])
                continue;
            if (!collide(famA, fam_data[shapeB]))
                continue;
            if (!overlap(Amin, Amax, Bmin, Bmax))
                continue;
            if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size_vec, bins_per_axis, bin_number[index]) == false)
                continue;

            if (shapeB < shapeA) {
                uint t = shapeA;
                shapeA = shapeB;
                shapeB = t;
            }
            // the two indices of the shapes that make up the contact
            potential_contacts[offset + count] = ((long long)shapeA << 32 | (long long)shapeB);
            count++;
        }
    }
}
}
}
