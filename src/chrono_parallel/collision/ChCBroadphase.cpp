#include <algorithm>

#include <chrono_parallel/collision/ChCBroadphase.h>
#include "chrono_parallel/collision/ChCBroadphaseUtils.h"
#include "chrono_parallel/collision/ChCBroadphaseFunctions.h"

#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/sort.h>
#include <thrust/sequence.h>
#include <thrust/iterator/constant_iterator.h>

using thrust::transform;
using thrust::transform_reduce;

namespace chrono {
namespace collision {

// Determine the bounding box for the objects===============================================================

void ChCBroadphase::DetermineBoundingBox() {
  host_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
  host_vector<real3>& aabb_max = data_manager->host_data.aabb_max;
  // determine the bounds on the total space and subdivide based on the bins per axis
  bbox res(aabb_min[0], aabb_min[0]);
  bbox_transformation unary_op;
  bbox_reduction binary_op;
  res = thrust::transform_reduce(aabb_min.begin(), aabb_min.end(), unary_op, res, binary_op);
  res = thrust::transform_reduce(aabb_max.begin(), aabb_max.end(), unary_op, res, binary_op);
  data_manager->measures.collision.min_bounding_point = res.first;
  data_manager->measures.collision.max_bounding_point = res.second;
  data_manager->measures.collision.global_origin = res.first;

  LOG(TRACE) << "Minimum bounding point: (" << res.first.x << ", " << res.first.y << ", " << res.first.z << ")";
  LOG(TRACE) << "Maximum bounding point: (" << res.second.x << ", " << res.second.y << ", " << res.second.z << ")";
}

void ChCBroadphase::OffsetAABB() {
  host_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
  host_vector<real3>& aabb_max = data_manager->host_data.aabb_max;
  thrust::constant_iterator<real3> offset(data_manager->measures.collision.global_origin);
  thrust::transform(aabb_min.begin(), aabb_min.end(), offset, aabb_min.begin(), thrust::minus<real3>());
  thrust::transform(aabb_max.begin(), aabb_max.end(), offset, aabb_max.begin(), thrust::minus<real3>());
}

// Determine resolution of the top level grid
void ChCBroadphase::ComputeTopLevelResolution() {
  const real3& min_bounding_point = data_manager->measures.collision.min_bounding_point;
  const real3& max_bounding_point = data_manager->measures.collision.max_bounding_point;
  real3& bin_size = data_manager->measures.collision.bin_size;
  const real3& global_origin = data_manager->measures.collision.global_origin;
  const real density = data_manager->settings.collision.grid_density;

  int3& bins_per_axis = data_manager->settings.collision.bins_per_axis;

  // This is the extents of the space aka diameter
  real3 diagonal = (absolute(max_bounding_point - global_origin));
  // Compute the number of slices in this grid level
  if (data_manager->settings.collision.fixed_bins == false) {
    bins_per_axis = function_Compute_Grid_Resolution(num_shapes, diagonal, density);
  }
  bin_size = diagonal / R3(bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);
  LOG(TRACE) << "bins_per_axis: (" << bins_per_axis.x << ", " << bins_per_axis.y << ", " << bins_per_axis.z << ")";
  LOG(TRACE) << "bin_size: (" << bin_size.x << ", " << bin_size.y << ", " << bin_size.z << ")";

  // Store the inverse for use later
  inv_bin_size = 1.0 / bin_size;
}

void ChCBroadphase::FillStateData() {
  fam_data = data_manager->host_data.fam_rigid;
  obj_active = data_manager->host_data.active_rigid;
  obj_data_id = data_manager->host_data.id_rigid;

  fam_data.resize(num_rigid_shapes + num_fluid_bodies);
  // individual shapes arent active/inactive the entire body is
  obj_active.resize(num_rigid_bodies + num_fluid_bodies);
  obj_data_id.resize(num_rigid_shapes + num_fluid_bodies);
  // set fluid family to the default
  thrust::fill(fam_data.begin() + num_rigid_shapes, fam_data.end(), S2(1, 0x7FFF));
  // individual shapes arent active/inactive the entire body is
  thrust::fill(obj_active.begin() + num_rigid_bodies, obj_active.end(), 1);
  // obj data id's go from 0->num_rigid_bodies->num_fluid so start at num_rigid_bodies
  thrust::sequence(obj_data_id.begin() + num_rigid_shapes, obj_data_id.end(), num_rigid_bodies);
}

// =========================================================================================================
ChCBroadphase::ChCBroadphase() {
  num_shapes = 0;
  number_of_contacts_possible = 0;
  num_bins_active = 0;
  number_of_bin_intersections = 0;
  number_of_leaf_intersections = 0;
  num_active_leaves = 0;
}
// =========================================================================================================
// use spatial subdivision to detect the list of POSSIBLE collisions
// let user define their own narrow-phase collision detection
void ChCBroadphase::DetectPossibleCollisions() {
  num_rigid_shapes = data_manager->num_rigid_shapes;
  num_rigid_bodies = data_manager->num_rigid_bodies;
  num_fluid_bodies = data_manager->num_fluid_bodies;
  num_shapes = num_rigid_shapes + num_fluid_bodies;
  LOG(TRACE) << "Number of AABBs: " << num_shapes;
  DetermineBoundingBox();
  OffsetAABB();
  ComputeTopLevelResolution();
  FillStateData();

  if (!data_manager->settings.collision.use_two_level) {
    OneLevelBroadphase();
  } else {
    TwoLevelBroadphase();
  }
  SplitContacts();

  return;
}

void ChCBroadphase::OneLevelBroadphase() {
  const host_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
  const host_vector<real3>& aabb_max = data_manager->host_data.aabb_max;
  host_vector<long long>& contact_pairs = data_manager->host_data.contact_pairs;
  int3& bins_per_axis = data_manager->settings.collision.bins_per_axis;

  bins_intersected.resize(num_shapes + 1);
  bins_intersected[num_shapes] = 0;

#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    f_Count_AABB_BIN_Intersection(i, inv_bin_size, aabb_min, aabb_max, bins_intersected);
  }

  Thrust_Exclusive_Scan(bins_intersected);
  number_of_bin_intersections = bins_intersected.back();

  LOG(TRACE) << "Number of bin intersections: " << number_of_bin_intersections;

  bin_number.resize(number_of_bin_intersections);
  bin_number_out.resize(number_of_bin_intersections);
  bin_aabb_number.resize(number_of_bin_intersections);
  bin_start_index.resize(number_of_bin_intersections);

#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    f_Store_AABB_BIN_Intersection(i, bins_per_axis, inv_bin_size, aabb_min, aabb_max, bins_intersected, bin_number,
                                  bin_aabb_number);
  }

  LOG(TRACE) << "Completed (device_Store_AABB_BIN_Intersection)";

  Thrust_Sort_By_Key(bin_number, bin_aabb_number);
  num_bins_active = Run_Length_Encode(bin_number, bin_number_out, bin_start_index);

  if (num_bins_active <= 0) {
    number_of_contacts_possible = 0;
    return;
  }

  bin_start_index.resize(num_bins_active + 1);
  bin_start_index[num_bins_active] = 0;

  LOG(TRACE) << bins_per_axis.x << " " << bins_per_axis.y << " " << bins_per_axis.z;
  LOG(TRACE) << "num_bins_active: " << num_bins_active;

  Thrust_Exclusive_Scan(bin_start_index);
  num_contact.resize(num_bins_active + 1);
  num_contact[num_bins_active] = 0;

#pragma omp parallel for
  for (int i = 0; i < num_bins_active; i++) {
    f_Count_AABB_AABB_Intersection(i, aabb_min, aabb_max, bin_number_out, bin_aabb_number, bin_start_index, fam_data,
                                   obj_active, obj_data_id, num_contact);
  }

  thrust::exclusive_scan(num_contact.begin(), num_contact.end(), num_contact.begin());
  number_of_contacts_possible = num_contact.back();
  contact_pairs.resize(number_of_contacts_possible);
  LOG(TRACE) << "Number of possible collisions: " << number_of_contacts_possible;

#pragma omp parallel for
  for (int index = 0; index < num_bins_active; index++) {
    f_Store_AABB_AABB_Intersection(index, aabb_min, aabb_max, bin_number_out, bin_aabb_number, bin_start_index,
                                      num_contact, fam_data, obj_active, obj_data_id, contact_pairs);
  }
  LOG(TRACE) << "Thrust_Sort(contact_pairs);: ";
  Thrust_Sort(contact_pairs);
  LOG(TRACE) << "Thrust_Unique(contact_pairs);: ";
  number_of_contacts_possible = Thrust_Unique(contact_pairs);
  contact_pairs.resize(number_of_contacts_possible);
  LOG(TRACE) << "Number of unique collisions: " << number_of_contacts_possible;
}
//======
void ChCBroadphase::TwoLevelBroadphase() {
  const host_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
  const host_vector<real3>& aabb_max = data_manager->host_data.aabb_max;

  int3 bins_per_axis = data_manager->settings.collision.bins_per_axis;
  real3 bin_size = data_manager->measures.collision.bin_size;
  // =========================================================================================================

  bins_intersected.resize(num_shapes + 1);
  bins_intersected[num_shapes] = 0;

// Determine AABB to top level bin count
#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    f_Count_AABB_BIN_Intersection(i, inv_bin_size, aabb_min, aabb_max, bins_intersected);
  }
  Thrust_Exclusive_Scan(bins_intersected);

  number_of_bin_intersections = bins_intersected.back();
  LOG(TRACE) << "number_of_bin_intersections: " << number_of_bin_intersections;
  // Allocate our AABB bin pairs==============================================================================
  bin_number.resize(number_of_bin_intersections);
  bin_number_out.resize(number_of_bin_intersections);
  bin_aabb_number.resize(number_of_bin_intersections);
  bin_start_index.resize(number_of_bin_intersections);

// Store the bin intersections================================================================================
#pragma omp parallel for
  for (int i = 0; i < num_shapes; i++) {
    f_Store_AABB_BIN_Intersection(i, bins_per_axis, inv_bin_size, aabb_min, aabb_max, bins_intersected, bin_number,
                                  bin_aabb_number);
  }

  // Get sorted top level intersections=======================================================================
  Thrust_Sort_By_Key(bin_number, bin_aabb_number);
  // Number of Top level Intersections========================================================================
  num_bins_active = Run_Length_Encode(bin_number, bin_number_out, bin_start_index);
  LOG(TRACE) << "num_bins_active: " << num_bins_active;
  // Extract Bin ranges=======================================================================================
  bin_start_index.resize(num_bins_active + 1);
  bin_start_index[num_bins_active] = 0;

  Thrust_Exclusive_Scan(bin_start_index);
  // Allocate space to hold leaf cell counts for each bin=====================================================
  leaves_per_bin.resize(num_bins_active + 1);
  leaves_per_bin[num_bins_active] = 0;
// Count leaves in each bin===================================================================================
#pragma omp parallel for
  for (int i = 0; i < num_bins_active; i++) {
    f_TL_Count_Leaves(i, data_manager->settings.collision.leaf_density, bin_size, bin_start_index, leaves_per_bin);
  }

  Thrust_Exclusive_Scan(leaves_per_bin);
  // Count leaf intersections=================================================================================
  leaves_intersected.resize(num_bins_active + 1);
  leaves_intersected[num_bins_active] = 0;
#pragma omp parallel for
  for (int i = 0; i < num_bins_active; i++) {
    f_TL_Count_AABB_Leaf_Intersection(i, data_manager->settings.collision.leaf_density, bin_size, bins_per_axis,
                                      bin_start_index, bin_number_out, bin_aabb_number, aabb_min, aabb_max,
                                      leaves_intersected);
  }

  Thrust_Exclusive_Scan(leaves_intersected);

  number_of_leaf_intersections = leaves_intersected.back();
  LOG(TRACE) << "number_of_leaf_intersections: " << number_of_leaf_intersections;

  leaf_number.resize(number_of_leaf_intersections);
  leaf_number_out.resize(number_of_leaf_intersections);
  leaf_aabb_number.resize(number_of_leaf_intersections);
  leaf_start_index.resize(number_of_leaf_intersections);
#pragma omp parallel for
  for (int i = 0; i < num_bins_active; i++) {
    f_TL_Write_AABB_Leaf_Intersection(i, data_manager->settings.collision.leaf_density, bin_size, bins_per_axis,
                                      bin_start_index, bin_number_out, bin_aabb_number, aabb_min, aabb_max,
                                      leaves_intersected, leaves_per_bin, leaf_number, leaf_aabb_number);
  }
  Thrust_Sort_By_Key(leaf_number, leaf_aabb_number);
  // Number of Leaf Intersections=============================================================================
  num_active_leaves = Run_Length_Encode(leaf_number, leaf_number_out, leaf_start_index);
  LOG(TRACE) << "num_active_leaves: " << num_active_leaves;

  // Extract leaf ranges======================================================================================
  leaf_start_index.resize(num_active_leaves + 1);
  leaf_start_index[num_active_leaves] = 0;

  Thrust_Exclusive_Scan(leaf_start_index);

  num_contact.resize(num_active_leaves + 1);
  num_contact[num_active_leaves] = 0;

#pragma omp parallel for
  for (int i = 0; i < num_active_leaves; i++) {
    f_Count_AABB_AABB_Intersection(i, aabb_min, aabb_max, leaf_number_out, leaf_aabb_number, leaf_start_index, fam_data,
                                   obj_active, obj_data_id, num_contact);
  }
  Thrust_Exclusive_Scan(num_contact);
  host_vector<long long>& contact_pairs = data_manager->host_data.contact_pairs;
  number_of_contacts_possible = num_contact.back();
  LOG(TRACE) << "number_of_contacts_possible: " << number_of_contacts_possible;
  contact_pairs.resize(number_of_contacts_possible);
  if (number_of_contacts_possible <= 0) {
    return;
  }

#pragma omp parallel for
  for (int i = 0; i < num_active_leaves; i++) {
    f_Store_AABB_AABB_Intersection(i, aabb_min, aabb_max, leaf_number_out, leaf_aabb_number, leaf_start_index,
                                      num_contact, fam_data, obj_active, obj_data_id, contact_pairs);
  }

  thrust::stable_sort(thrust_parallel, contact_pairs.begin(), contact_pairs.end());
  number_of_contacts_possible = Thrust_Unique(contact_pairs);

  contact_pairs.resize(number_of_contacts_possible);
}
void ChCBroadphase::SplitContacts() {
  LOG(TRACE) << "ChCBroadphase::SplitContacts(): ";

  // Split Contacts into three lists

  const uint num_rigid_shapes = data_manager->num_rigid_shapes;
  const uint num_fluid_bodies = data_manager->num_fluid_bodies;
  host_vector<long long>& contact_pairs = data_manager->host_data.contact_pairs;
  LOG(TRACE) << "number_of_contacts_possible: " << number_of_contacts_possible;
  host_vector<int> contact_type(number_of_contacts_possible);
#pragma omp parallel for
  for (int i = 0; i < number_of_contacts_possible; i++) {
    int2 pair = I2(int(contact_pairs[i] >> 32), int(contact_pairs[i] & 0xffffffff));

    if (pair.x < num_rigid_shapes && pair.y < num_rigid_shapes) {
      contact_type[i] = 0;
    } else if (pair.x < num_rigid_shapes && pair.y >= num_rigid_shapes) {
      contact_type[i] = 1;
    } else if (pair.x >= num_rigid_shapes && pair.y >= num_rigid_shapes) {
      contact_type[i] = 2;
    } else {
      contact_type[i] = 3;
    }
  }
  LOG(TRACE) << "Thrust_Sort_By_Key(contact_type, contact_pairs);";
  Thrust_Sort_By_Key(contact_type, contact_pairs);
  LOG(TRACE) << "Thrust_Count(contact_type,...)";
  data_manager->num_rigid_contacts = Thrust_Count(contact_type, 0);
  data_manager->num_rigid_fluid_contacts = Thrust_Count(contact_type, 1);
  data_manager->num_fluid_contacts = Thrust_Count(contact_type, 2);

  LOG(TRACE) << "num_rigid_contacts: " << data_manager->num_rigid_contacts;
  LOG(TRACE) << "num_rigid_fluid_contacts: " << data_manager->num_rigid_fluid_contacts;
  LOG(TRACE) << "num_fluid_contacts: " << data_manager->num_fluid_contacts;
  LOG(TRACE) << "total contacts in broadphase: " << number_of_contacts_possible;
}
}
}
