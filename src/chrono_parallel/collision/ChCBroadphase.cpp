#include <algorithm>

#include <chrono_parallel/collision/ChCBroadphase.h>
#include "chrono_parallel/collision/ChCBroadphaseUtils.h"

#include <thrust/transform.h>
#include <thrust/iterator/constant_iterator.h>


using thrust::transform;
using thrust::transform_reduce;

namespace chrono {
namespace collision {

// Function to Count AABB Bin intersections=================================================================
inline void function_Count_AABB_BIN_Intersection(const uint index,
                                                 const real3& inv_bin_size_vec,
                                                 const host_vector<real3>& aabb_min_data,
                                                 const host_vector<real3>& aabb_max_data,
                                                 host_vector<uint>& bins_intersected) {
  int3 gmin = HashMin(aabb_min_data[index], inv_bin_size_vec);
  int3 gmax = HashMax(aabb_max_data[index], inv_bin_size_vec);
  bins_intersected[index] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
}

// Function to Store AABB Bin Intersections=================================================================
inline void function_Store_AABB_BIN_Intersection(const uint index,
                                                 const int3& bins_per_axis,
                                                 const real3& inv_bin_size_vec,
                                                 const host_vector<real3>& aabb_min_data,
                                                 const host_vector<real3>& aabb_max_data,
                                                 const host_vector<uint>& bins_intersected,
                                                 host_vector<uint>& bin_number,
                                                 host_vector<uint>& aabb_number) {
  uint count = 0, i, j, k;
  int3 gmin = HashMin(aabb_min_data[index], inv_bin_size_vec);
  int3 gmax = HashMax(aabb_max_data[index], inv_bin_size_vec);
  uint mInd = bins_intersected[index];
  for (i = gmin.x; i <= gmax.x; i++) {
    for (j = gmin.y; j <= gmax.y; j++) {
      for (k = gmin.z; k <= gmax.z; k++) {
        bin_number[mInd + count] = Hash_Index(I3(i, j, k), bins_per_axis);
        aabb_number[mInd + count] = index;
        count++;
      }
    }
  }
}

// Function to count AABB AABB intersection=================================================================
inline void function_Count_AABB_AABB_Intersection(const uint index,
                                                  const real3 inv_bin_size_vec,
                                                  const int3 bins_per_axis,
                                                  const host_vector<real3>& aabb_min_data,
                                                  const host_vector<real3>& aabb_max_data,
                                                  const host_vector<uint>& bin_number,
                                                  const host_vector<uint>& aabb_number,
                                                  const host_vector<uint>& bin_start_index,
                                                  const host_vector<short2>& fam_data,
                                                  const host_vector<bool>& body_active,
                                                  const host_vector<uint>& body_id,
                                                  host_vector<uint>& num_contact) {
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

      if(current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size_vec, bins_per_axis, bin_number[index])==false)
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
inline void function_Store_AABB_AABB_Intersection(const uint index,
                                                  const real3 inv_bin_size_vec,
                                                  const int3 bins_per_axis,
                                                  const host_vector<real3>& aabb_min_data,
                                                  const host_vector<real3>& aabb_max_data,
                                                  const host_vector<uint>& bin_number,
                                                  const host_vector<uint>& aabb_number,
                                                  const host_vector<uint>& bin_start_index,
                                                  const host_vector<uint>& num_contact,
                                                  const host_vector<short2>& fam_data,
                                                  const host_vector<bool>& body_active,
                                                  const host_vector<uint>& body_id,
                                                  host_vector<long long>& potential_contacts) {
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

      if(current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size_vec, bins_per_axis, bin_number[index])==false)
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
// =========================================================================================================
ChCBroadphase::ChCBroadphase() {
  number_of_contacts_possible = 0;
  num_bins_active = 0;
  number_of_bin_intersections = 0;
  data_manager = 0;
}
// =========================================================================================================
// use spatial subdivision to detect the list of POSSIBLE collisions
// let user define their own narrow-phase collision detection
void ChCBroadphase::DetectPossibleCollisions() {
  host_vector<real3>& aabb_min_rigid = data_manager->host_data.aabb_min_rigid;
  host_vector<real3>& aabb_max_rigid = data_manager->host_data.aabb_max_rigid;

  host_vector<long long>& contact_pairs = data_manager->host_data.pair_rigid_rigid;
  real3& min_bounding_point = data_manager->measures.collision.min_bounding_point;
  real3& max_bounding_point = data_manager->measures.collision.max_bounding_point;
  real3& bin_size_vec = data_manager->measures.collision.bin_size_vec;
  real3& global_origin = data_manager->measures.collision.global_origin;
  int3& bins_per_axis = data_manager->settings.collision.bins_per_axis;
  const real density = data_manager->settings.collision.grid_density;
  const host_vector<short2>& fam_data = data_manager->host_data.fam_rigid;
  const host_vector<bool>& obj_active = data_manager->host_data.active_rigid;
  const host_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;
  uint num_shapes = data_manager->num_rigid_shapes;

  LOG(TRACE) << "Number of AABBs: " << num_shapes;
  contact_pairs.clear();
  // STEP 2: determine the bounds on the total space and subdivide based on the bins per axis
  // create a zero volume bounding box using the first aabb
  bbox res = bbox(aabb_min_rigid[0], aabb_min_rigid[0]);
  bbox_transformation unary_op;
  bbox_reduction binary_op;
  // Grow the initial bounding box to contain all of the aabbs
  res = transform_reduce(thrust_parallel, aabb_min_rigid.begin(), aabb_min_rigid.end(), unary_op, res, binary_op);
  res = transform_reduce(thrust_parallel, aabb_max_rigid.begin(), aabb_max_rigid.end(), unary_op, res, binary_op);
  min_bounding_point = res.first;
  max_bounding_point = res.second;
  global_origin = min_bounding_point;
  real3 diagonal = max_bounding_point - min_bounding_point;

  if (data_manager->settings.collision.fixed_bins == false) {
    bins_per_axis = function_Compute_Grid_Resolution(num_shapes, diagonal, density);
  }
  bin_size_vec = diagonal / R3(bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);
  real3 inv_bin_size_vec = 1.0 / bin_size_vec;

  thrust::constant_iterator<real3> offset(global_origin);
  transform(aabb_min_rigid.begin(), aabb_min_rigid.end(), offset, aabb_min_rigid.begin(), thrust::minus<real3>());
  transform(aabb_max_rigid.begin(), aabb_max_rigid.end(), offset, aabb_max_rigid.begin(), thrust::minus<real3>());

  LOG(TRACE) << "Minimum bounding point: (" << res.first.x << ", " << res.first.y << ", " << res.first.z << ")";
  LOG(TRACE) << "Maximum bounding point: (" << res.second.x << ", " << res.second.y << ", " << res.second.z << ")";
  LOG(TRACE) << "Bin size vector: (" << bin_size_vec.x << ", " << bin_size_vec.y << ", " << bin_size_vec.z << ")";

  bins_intersected.resize(num_shapes + 1);
  bins_intersected[num_shapes] = 0;

#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    function_Count_AABB_BIN_Intersection(i, inv_bin_size_vec, aabb_min_rigid, aabb_max_rigid, bins_intersected);
  }

  Thrust_Exclusive_Scan(bins_intersected);
  number_of_bin_intersections = bins_intersected.back();

  LOG(TRACE) << "Number of bin intersections: " << number_of_bin_intersections;

  bin_number.resize(number_of_bin_intersections);
  bin_number_out.resize(number_of_bin_intersections);
  aabb_number.resize(number_of_bin_intersections);
  bin_start_index.resize(number_of_bin_intersections);

#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    function_Store_AABB_BIN_Intersection(i, bins_per_axis, inv_bin_size_vec, aabb_min_rigid, aabb_max_rigid,
                                         bins_intersected, bin_number, aabb_number);
  }

  LOG(TRACE) << "Completed (device_Store_AABB_BIN_Intersection)";

  Thrust_Sort_By_Key(bin_number, aabb_number);
  num_bins_active = Run_Length_Encode(bin_number, bin_number_out, bin_start_index);

  if (num_bins_active <= 0) {
    number_of_contacts_possible = 0;
    return;
  }

  bin_start_index.resize(num_bins_active + 1);
  bin_start_index[num_bins_active] = 0;

  LOG(TRACE) << bins_per_axis.x << " " << bins_per_axis.y << " " << bins_per_axis.z;
  LOG(TRACE) << "Last active bin: " << num_bins_active;

  Thrust_Exclusive_Scan(bin_start_index);
  num_contact.resize(num_bins_active + 1);
  num_contact[num_bins_active] = 0;

#pragma omp parallel for
  for (int i = 0; i < num_bins_active; i++) {
    function_Count_AABB_AABB_Intersection(
      i, 
      inv_bin_size_vec,
      bins_per_axis,
      aabb_min_rigid, 
      aabb_max_rigid, 
      bin_number_out, 
      aabb_number, 
      bin_start_index,
      fam_data, 
      obj_active, 
      obj_data_ID, 
      num_contact);
  }
  Thrust_Exclusive_Scan(num_contact);
  number_of_contacts_possible = num_contact.back();
  contact_pairs.resize(number_of_contacts_possible);
  LOG(TRACE) << "Number of possible collisions: " << number_of_contacts_possible;

#pragma omp parallel for
  for (int index = 0; index < num_bins_active; index++) {
    function_Store_AABB_AABB_Intersection(index, 
      inv_bin_size_vec,
      bins_per_axis,
      aabb_min_rigid, 
      aabb_max_rigid, 
      bin_number_out, 
      aabb_number,
      bin_start_index, 
      num_contact, 
      fam_data, 
      obj_active, 
      obj_data_ID,
      contact_pairs);
  }

  contact_pairs.resize(number_of_contacts_possible);

  LOG(TRACE) << "Number of possible collisions: " << number_of_contacts_possible;

  return;
}
}
}
