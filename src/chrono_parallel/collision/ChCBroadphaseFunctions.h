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
// Authors: Hammad Mazhar
// =============================================================================
// Functions used by both broadphase algorithms
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCAABBGenerator.h"

namespace chrono {
namespace collision {

// TWO LEVEL FUNCTIONS==========================================================

// For each bin determine the grid size and store it========================================================
void f_TL_Count_Leaves(const uint index,
                       const real density,
                       const real3& bin_size,
                       const custom_vector<uint>& bin_start_index,
                       custom_vector<uint>& leaves_per_bin) {
    uint start = bin_start_index[index];
    uint end = bin_start_index[index + 1];
    uint num_aabb_in_cell = end - start;

    int3 cell_res = function_Compute_Grid_Resolution(num_aabb_in_cell, bin_size, density);

    leaves_per_bin[index] = cell_res.x * cell_res.y * cell_res.z;
}
// Count the number of AABB leaf intersections for each bin=================================================
void f_TL_Count_AABB_Leaf_Intersection(const uint index,
                                       const real density,
                                       const real3& bin_size,
                                       const int3& bins_per_axis,
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
    int3 cell_res = function_Compute_Grid_Resolution(num_aabb_in_cell, bin_size, density);
    real3 inv_leaf_size = real3(cell_res.x, cell_res.y, cell_res.z) / bin_size;
    int3 bin_index = Hash_Decode(bin_number[index], bins_per_axis);
    real3 bin_position = real3(bin_index.x * bin_size.x, bin_index.y * bin_size.y, bin_index.z * bin_size.z);

    for (uint i = start; i < end; i++) {
        uint shape = shape_number[i];
        // subtract the bin position from the AABB position
        real3 Amin = aabb_min[shape] - bin_position;
        real3 Amax = aabb_max[shape] - bin_position;

        // Make sure that even with subtraction we are at the origin
        Amin = Clamp(Amin, real3(0), Amax);

        // Find the extents
        int3 gmin = HashMin(Amin, inv_leaf_size);
        int3 gmax = HashMax(Amax, inv_leaf_size);
        // Make sure that the maximum bin value does not exceed the bounds of this grid
        int3 max_clamp = cell_res - int3(1);
        gmin = Clamp(gmin, int3(0), max_clamp);
        gmax = Clamp(gmax, int3(0), max_clamp);

        count += (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }

    leaves_intersected[index] = count;
}
// Store the AABB leaf intersections for each bin===========================================================
void f_TL_Write_AABB_Leaf_Intersection(const uint& index,
                                       const real density,
                                       const real3& bin_size,
                                       const int3& bin_resolution,
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
    int3 cell_res = function_Compute_Grid_Resolution(num_aabb_in_cell, bin_size, density);
    real3 inv_leaf_size = real3(cell_res.x, cell_res.y, cell_res.z) / bin_size;

    int3 bin_index = Hash_Decode(bin_number[index], bin_resolution);

    real3 bin_position = real3(bin_index.x * bin_size.x, bin_index.y * bin_size.y, bin_index.z * bin_size.z);

    for (uint i = start; i < end; i++) {
        uint shape = bin_shape_number[i];
        // subtract the bin position from the AABB position
        real3 Amin = aabb_min[shape] - bin_position;
        real3 Amax = aabb_max[shape] - bin_position;

        // Make sure that even with subtraction we are at the origin
        Amin = Clamp(Amin, real3(0), Amax);

        // Find the extents
        int3 gmin = HashMin(Amin, inv_leaf_size);
        int3 gmax = HashMax(Amax, inv_leaf_size);

        // Make sure that the maximum bin value does not exceed the bounds of this grid
        int3 max_clamp = cell_res - int3(1);
        gmin = Clamp(gmin, int3(0), max_clamp);
        gmax = Clamp(gmax, int3(0), max_clamp);

        int a, b, c;
        for (a = gmin.x; a <= gmax.x; a++) {
            for (b = gmin.y; b <= gmax.y; b++) {
                for (c = gmin.z; c <= gmax.z; c++) {
                    leaf_number[mInd + count] = leaves_per_bin[index] + Hash_Index(int3(a, b, c), cell_res);
                    leaf_shape_number[mInd + count] = shape;
                    count++;
                }
            }
        }
    }
}

// ONE AND TWO LEVEL FUNCTIONS==========================================================

// Function to Count AABB Bin intersections=================================================================
inline void f_Count_AABB_BIN_Intersection(const uint index,
                                          const real3& inv_bin_size,
                                          const custom_vector<real3>& aabb_min,
                                          const custom_vector<real3>& aabb_max,
                                          custom_vector<uint>& bins_intersected) {
    int3 gmin = HashMin(aabb_min[index], inv_bin_size);
    int3 gmax = HashMax(aabb_max[index], inv_bin_size);
    bins_intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
}

// Function to Store AABB Bin Intersections=================================================================
inline void f_Store_AABB_BIN_Intersection(const uint index,
                                          const int3& bins_per_axis,
                                          const real3& inv_bin_size,
                                          const custom_vector<real3>& aabb_min_data,
                                          const custom_vector<real3>& aabb_max_data,
                                          const custom_vector<uint>& bins_intersected,
                                          custom_vector<uint>& bin_number,
                                          custom_vector<uint>& aabb_number) {
    uint count = 0, i, j, k;
    int3 gmin = HashMin(aabb_min_data[index], inv_bin_size);
    int3 gmax = HashMax(aabb_max_data[index], inv_bin_size);
    uint mInd = bins_intersected[index];
    for (i = gmin.x; i <= gmax.x; i++) {
        for (j = gmin.y; j <= gmax.y; j++) {
            for (k = gmin.z; k <= gmax.z; k++) {
                bin_number[mInd + count] = Hash_Index(int3(i, j, k), bins_per_axis);
                aabb_number[mInd + count] = index;
                count++;
            }
        }
    }
}

// Function to count AABB AABB intersection=================================================================
inline void f_Count_AABB_AABB_Intersection(const uint index,
                                           const real3 inv_bin_size_vec,
                                           const int3 bins_per_axis,
                                           const custom_vector<real3>& aabb_min_data,
                                           const custom_vector<real3>& aabb_max_data,
                                           const custom_vector<uint>& bin_number,
                                           const custom_vector<uint>& aabb_number,
                                           const custom_vector<uint>& bin_start_index,
                                           const custom_vector<short2>& fam_data,
                                           const custom_vector<char>& body_active,
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

        for (uint k = i + 1; k < end; k++) {
            uint shapeB = aabb_number[k];
            uint bodyB = body_id[shapeB];
            real3 Bmin = aabb_min_data[shapeB];
            real3 Bmax = aabb_max_data[shapeB];

            if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size_vec, bins_per_axis, bin_number[index]) == false)
                continue;
            if (shapeA == shapeB)
                continue;
            if (bodyA == bodyB)
                continue;
            if (!body_active[bodyA] && !body_active[bodyB])
                continue;
            if (!collide(famA, fam_data[shapeB]))
                continue;
            if (!overlap(Amin, Amax, Bmin, Bmax))
                continue;
            count++;
        }
    }

    num_contact[index] = count;
}

// Function to store AABB-AABB intersections================================================================
inline void f_Store_AABB_AABB_Intersection(const uint index,
                                           const real3 inv_bin_size_vec,
                                           const int3 bins_per_axis,
                                           const custom_vector<real3>& aabb_min_data,
                                           const custom_vector<real3>& aabb_max_data,
                                           const custom_vector<uint>& bin_number,
                                           const custom_vector<uint>& aabb_number,
                                           const custom_vector<uint>& bin_start_index,
                                           const custom_vector<uint>& num_contact,
                                           const custom_vector<short2>& fam_data,
                                           const custom_vector<char>& body_active,
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

        for (int k = i + 1; k < end; k++) {
            uint shapeB = aabb_number[k];
            uint bodyB = body_id[shapeB];
            real3 Bmin = aabb_min_data[shapeB];
            real3 Bmax = aabb_max_data[shapeB];

            if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size_vec, bins_per_axis, bin_number[index]) == false)
                continue;
            if (shapeA == shapeB)
                continue;
            if (bodyA == bodyB)
                continue;
            if (!body_active[bodyA] && !body_active[bodyB])
                continue;
            if (!collide(famA, fam_data[shapeB]))
                continue;
            if (!overlap(Amin, Amax, Bmin, Bmax))
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
