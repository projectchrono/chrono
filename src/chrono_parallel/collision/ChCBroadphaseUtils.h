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
// Utility functions used by both broadphase algorithms
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCAABBGenerator.h"

namespace chrono {
namespace collision {

typedef thrust::pair<real3, real3> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction : public thrust::binary_function<bbox, bbox, bbox> {
  bbox operator()(bbox a, bbox b) {
    real3 ll = R3(Min(a.first.x, b.first.x), Min(a.first.y, b.first.y),
                  Min(a.first.z, b.first.z));  // lower left corner
    real3 ur = R3(Max(a.second.x, b.second.x), Max(a.second.y, b.second.y),
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
inline int3 HashMin(const T& A, const real3& inv_bin_size_vec) {
  int3 temp;
  temp.x = floor(A.x * inv_bin_size_vec.x);
  temp.y = floor(A.y * inv_bin_size_vec.y);
  temp.z = floor(A.z * inv_bin_size_vec.z);
  return temp;
}

template <class T>
inline int3 HashMax(const T& A, const real3& inv_bin_size_vec) {
  int3 temp;
  temp.x = std::ceil(A.x * inv_bin_size_vec.x) - 1;
  temp.y = std::ceil(A.y * inv_bin_size_vec.y) - 1;
  temp.z = std::ceil(A.z * inv_bin_size_vec.z) - 1;
  return temp;
}

// Convert a bin index into a unique hash value
inline uint Hash_Index(const int3& A, int3 bins_per_axis) {
  return ((A.z * bins_per_axis.y) * bins_per_axis.x) + (A.y * bins_per_axis.x) + A.x;
}
// Decodes a hash into it's associated bin position
inline int3 Hash_Decode(uint hash, int3 bins_per_axis) {
  int3 decoded_hash;
  decoded_hash.x = hash % (bins_per_axis.x * bins_per_axis.y) % bins_per_axis.x;
  decoded_hash.y = (hash % (bins_per_axis.x * bins_per_axis.y)) / bins_per_axis.x;
  decoded_hash.z = hash / (bins_per_axis.x * bins_per_axis.y);
  return decoded_hash;
}

// AABB COLLISION FUNCTIONS ================================================================================

// Check if two bodies interact using their collision family data.
inline bool collide(short2 fam_data_A, short2 fam_data_B) {
  // Return true only if the bit corresponding to family of B is set in the mask
  // of A and vice-versa.
  return (fam_data_A.y & fam_data_B.x) && (fam_data_B.y & fam_data_A.x);
}

// Check if two AABBs overlap using their min/max corners.
inline bool overlap(real3 Amin, real3 Amax, real3 Bmin, real3 Bmax) {
  // Return true only if the two AABBs overlap in all 3 directions.
  return (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
         (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
}

inline bool current_bin(real3 Amin, real3 Amax, real3 Bmin, real3 Bmax, real3 inv_bin_size_vec, int3 bins_per_axis, uint bin) {
  real3 min_p;
  min_p.x = std::max(Amin.x,Bmin.x);
  min_p.y = std::max(Amin.y,Bmin.y);
  min_p.z = std::max(Amin.z,Bmin.z);

  real3 max_p;
  max_p.x = std::min(Amax.x,Bmax.x);
  max_p.y = std::min(Amax.y,Bmax.y);
  max_p.z = std::min(Amax.z,Bmax.z);

  real3 center = (min_p+max_p) * 0.5; //center of contact volume

  if(Hash_Index(HashMin(min_p, inv_bin_size_vec), bins_per_axis)==bin){
    return true;
  }
return false;
}

// Grid Size FUNCTIONS =====================================================================================

// for a given number of aabbs in a grid, the grids maximum and minimum point along with the density factor
// compute the size of the grid
static int3 function_Compute_Grid_Resolution(uint num_aabb, real3 d, real k = .1) {
  int3 grid_size = I3(0);
  real V = d.x * d.y * d.z;
  grid_size.x = int(d.x * std::pow(k * num_aabb / V, 1.0 / 3.0));
  grid_size.y = int(d.y * std::pow(k * num_aabb / V, 1.0 / 3.0));
  grid_size.z = int(d.z * std::pow(k * num_aabb / V, 1.0 / 3.0));

  grid_size = clamp(grid_size, I3(1), grid_size);

  return grid_size;
}

// =========================================================================================================

bool function_Check_Sphere(real3 pos_a, real3 pos_b, real radius) {
  real3 delta = pos_b - pos_a;
  real dist2 = dot(delta, delta);
  real radSum = radius + radius;
  if (dist2 >= radSum * radSum || dist2 < 1e-12) {
    return false;
  }
  return true;
}

}
}

