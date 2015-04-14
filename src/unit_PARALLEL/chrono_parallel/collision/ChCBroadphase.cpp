#include <algorithm>

// not used but prevents compilation errors with cuda 7 RC
#include <thrust/transform.h>

#include <thrust/iterator/constant_iterator.h>
#include <chrono_parallel/collision/ChCBroadphase.h>
#include "chrono_parallel/collision/ChCBroadphaseUtils.h"
namespace chrono {
namespace collision {

// Function to Count AABB Bin intersections=================================================================
inline void function_Count_AABB_BIN_Intersection(const uint index,
                                                 const uint num_shapes,
                                                 const real3* aabb_data,
                                                 const real3& inv_bin_size_vec,
                                                 uint* bins_intersected) {
  int3 gmin = HashMin(aabb_data[index], inv_bin_size_vec);
  int3 gmax = HashMax(aabb_data[index + num_shapes], inv_bin_size_vec);
  bins_intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
}

// Function to Store AABB Bin Intersections=================================================================
inline void function_Store_AABB_BIN_Intersection(const uint index,
                                                 const uint num_shapes,
                                                 const int3& bins_per_axis,
                                                 const real3& inv_bin_size_vec,
                                                 const real3* aabb_data,
                                                 const uint* bins_intersected,
                                                 uint* bin_number,
                                                 uint* shape_number) {
  uint count = 0, i, j, k;
  int3 gmin = HashMin(aabb_data[index], inv_bin_size_vec);
  int3 gmax = HashMax(aabb_data[index + num_shapes], inv_bin_size_vec);
  uint mInd = bins_intersected[index];
  for (i = gmin.x; i <= gmax.x; i++) {
    for (j = gmin.y; j <= gmax.y; j++) {
      for (k = gmin.z; k <= gmax.z; k++) {
        bin_number[mInd + count] = Hash_Index(I3(i, j, k), bins_per_axis);
        shape_number[mInd + count] = index;
        count++;
      }
    }
  }
}

// Function to count AABB AABB intersection=================================================================
inline void function_Count_AABB_AABB_Intersection(const uint index,
                                                  const uint num_shapes,
                                                  const real3* aabb_data,
                                                  const uint* bin_number,
                                                  const uint* shape_number,
                                                  const uint* bin_start_index,
                                                  const short2* fam_data,
                                                  const bool* body_active,
                                                  const uint* body_id,
                                                  uint* num_contact) {
  uint start = bin_start_index[index];
  uint end = bin_start_index[index + 1];
  uint count = 0;
  // Terminate early if there is only one object in the bin
  if (end - start == 1) {
    num_contact[index] = 0;
    return;
  }
  for (uint i = start; i < end; i++) {
    uint shapeA = shape_number[i];
    real3 Amin = aabb_data[shapeA];
    real3 Amax = aabb_data[shapeA + num_shapes];
    short2 famA = fam_data[shapeA];
    uint bodyA = body_id[shapeA];

    for (uint k = i + 1; k < end; k++) {
      uint shapeB = shape_number[k];
      uint bodyB = body_id[shapeB];

      if (shapeA == shapeB)
        continue;
      if (bodyA == bodyB)
        continue;
      if (!body_active[bodyA] && !body_active[bodyB])
        continue;
      if (!collide(famA, fam_data[shapeB]))
        continue;
      if (!overlap(Amin, Amax, aabb_data[shapeB], aabb_data[shapeB + num_shapes]))
        continue;
      count++;
    }
  }

  num_contact[index] = count;
}

// Function to store AABB-AABB intersections================================================================
inline void function_Store_AABB_AABB_Intersection(const uint index,
                                                  const uint num_shapes,
                                                  const real3* aabb_data,
                                                  const uint* bin_number,
                                                  const uint* shape_number,
                                                  const uint* bin_start_index,
                                                  const uint* num_contact,
                                                  const short2* fam_data,
                                                  const bool* body_active,
                                                  const uint* body_id,
                                                  long long* potential_contacts) {
  uint start = bin_start_index[index];
  uint end = bin_start_index[index + 1];
  // Terminate early if there is only one object in the bin
  if (end - start == 1) {
    return;
  }
  uint offset = num_contact[index];
  uint count = 0;

  for (uint i = start; i < end; i++) {
    uint shapeA = shape_number[i];
    real3 Amin = aabb_data[shapeA];
    real3 Amax = aabb_data[shapeA + num_shapes];
    short2 famA = fam_data[shapeA];
    uint bodyA = body_id[shapeA];

    for (int k = i + 1; k < end; k++) {
      uint shapeB = shape_number[k];
      uint bodyB = body_id[shapeB];

      if (shapeA == shapeB)
        continue;
      if (bodyA == bodyB)
        continue;
      if (!body_active[bodyA] && !body_active[bodyB])
        continue;
      if (!collide(famA, fam_data[shapeB]))
        continue;
      if (!overlap(Amin, Amax, aabb_data[shapeB], aabb_data[shapeB + num_shapes]))
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
// =========================================================================================================
ChCBroadphase::ChCBroadphase() {
  number_of_contacts_possible = 0;
  last_active_bin = 0;
  number_of_bin_intersections = 0;
  num_shapes = 0;
}
// =========================================================================================================
// use spatial subdivision to detect the list of POSSIBLE collisions
// let user define their own narrow-phase collision detection
void ChCBroadphase::DetectPossibleCollisions() {
  custom_vector<real3>& aabb_data = data_manager->host_data.aabb_rigid;
  custom_vector<long long>& contact_pairs = data_manager->host_data.pair_rigid_rigid;
  real3& min_bounding_point = data_manager->measures.collision.min_bounding_point;
  real3& max_bounding_point = data_manager->measures.collision.max_bounding_point;
  real3& bin_size_vec = data_manager->measures.collision.bin_size_vec;
  real3& global_origin = data_manager->measures.collision.global_origin;
  int3& bins_per_axis = data_manager->settings.collision.bins_per_axis;
  const real density = 5;  // data_manager->settings.collision.grid_density;
  const custom_vector<short2>& fam_data = data_manager->host_data.fam_rigid;
  const custom_vector<bool>& obj_active = data_manager->host_data.active_rigid;
  const custom_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;
  num_shapes = aabb_data.size() / 2;

  LOG(TRACE) << "Number of AABBs: " << num_shapes;
  contact_pairs.clear();
  // STEP 2: determine the bounds on the total space and subdivide based on the bins per axis
  // create a zero volume bounding box using the first aabb
  bbox init = bbox(aabb_data[0], aabb_data[0]);
  bbox_transformation unary_op;
  bbox_reduction binary_op;
  // Grow the initial bounding box to contain all of the aabbs
  bbox res = thrust::transform_reduce(thrust_parallel, aabb_data.begin(), aabb_data.end(), unary_op, init, binary_op);
  min_bounding_point = res.first;
  max_bounding_point = res.second;
  global_origin = min_bounding_point;
  real3 diagonal = max_bounding_point - min_bounding_point;

  // if (data_manager->settings.collision.fixed_bins == false) {
  // bins_per_axis = function_Compute_Grid_Resolution(num_shapes, diagonal, density);
  //}
  bin_size_vec = diagonal / R3(bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);
  inv_bin_size_vec = 1.0 / bin_size_vec;

  thrust::constant_iterator<real3> offset(global_origin);
  thrust::transform(aabb_data.begin(), aabb_data.end(), offset, aabb_data.begin(), thrust::minus<real3>());

  LOG(TRACE) << "Minimum bounding point: (" << res.first.x << ", " << res.first.y << ", " << res.first.z << ")";
  LOG(TRACE) << "Maximum bounding point: (" << res.second.x << ", " << res.second.y << ", " << res.second.z << ")";
  LOG(TRACE) << "Bin size vector: (" << bin_size_vec.x << ", " << bin_size_vec.y << ", " << bin_size_vec.z << ")";

  bins_intersected.resize(num_shapes + 1);
  bins_intersected[num_shapes] = 0;

#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    function_Count_AABB_BIN_Intersection(i, num_shapes, aabb_data.data(), inv_bin_size_vec, bins_intersected.data());
  }

  Thrust_Exclusive_Scan(bins_intersected);
  number_of_bin_intersections = bins_intersected.back();

  LOG(TRACE) << "Number of bin intersections: " << number_of_bin_intersections;

  bin_number.resize(number_of_bin_intersections);
  shape_number.resize(number_of_bin_intersections);
  bin_start_index.resize(number_of_bin_intersections);

#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    function_Store_AABB_BIN_Intersection(i, num_shapes, bins_per_axis, inv_bin_size_vec, aabb_data.data(),
                                         bins_intersected.data(), bin_number.data(), shape_number.data());
  }

  LOG(TRACE) << "Completed (device_Store_AABB_BIN_Intersection)";

  Thrust_Sort_By_Key(bin_number, shape_number);
  last_active_bin = Thrust_Reduce_By_Key(bin_number, bin_number, bin_start_index);

  if (last_active_bin <= 0) {
    number_of_contacts_possible = 0;
    return;
  }

  bin_start_index.resize(last_active_bin + 1);
  bin_start_index[last_active_bin] = 0;

  LOG(TRACE) << bins_per_axis.x << " " << bins_per_axis.y << " " << bins_per_axis.z;
  LOG(TRACE) << "Last active bin: " << last_active_bin;

  Thrust_Exclusive_Scan(bin_start_index);
  num_contact.resize(last_active_bin + 1);
  num_contact[last_active_bin] = 0;

#pragma omp parallel for
  for (int i = 0; i < last_active_bin; i++) {
    function_Count_AABB_AABB_Intersection(i, num_shapes, aabb_data.data(), bin_number.data(), shape_number.data(),
                                          bin_start_index.data(), fam_data.data(), obj_active.data(),
                                          obj_data_ID.data(), num_contact.data());
  }

  thrust::exclusive_scan(num_contact.begin(), num_contact.end(), num_contact.begin());
  number_of_contacts_possible = num_contact.back();
  contact_pairs.resize(number_of_contacts_possible);
  LOG(TRACE) << "Number of possible collisions: " << number_of_contacts_possible;

#pragma omp parallel for
  for (int index = 0; index < last_active_bin; index++) {
    function_Store_AABB_AABB_Intersection(index, num_shapes, aabb_data.data(), bin_number.data(), shape_number.data(),
                                          bin_start_index.data(), num_contact.data(), fam_data.data(),
                                          obj_active.data(), obj_data_ID.data(), contact_pairs.data());
  }

  thrust::stable_sort(thrust_parallel, contact_pairs.begin(), contact_pairs.end());

  number_of_contacts_possible = Thrust_Unique(contact_pairs);

  contact_pairs.resize(number_of_contacts_possible);

  LOG(TRACE) << "Number of possible collisions: " << number_of_contacts_possible;

  return;
}
}
}
