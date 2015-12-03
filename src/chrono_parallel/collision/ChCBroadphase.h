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
// The boradphase algorithm uses a spatial subdivison approach to find contacts
// between objects of different sizes
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCAABBGenerator.h"

namespace chrono {
namespace collision {

class CH_PARALLEL_API ChCBroadphase {
 public:
  // functions
  ChCBroadphase();
  void DetectPossibleCollisions();
  void OneLevelBroadphase();
  void TwoLevelBroadphase();
  void DetermineBoundingBox();
  void OffsetAABB();
  void ComputeTopLevelResolution();
  void FillStateData();
  void SplitContacts();

  ChParallelDataManager* data_manager;
  uint num_shapes;
  real level_one_density;
  real lelel_two_density;

 private:
  uint num_bins_active;
  uint number_of_bin_intersections;
  uint number_of_contacts_possible;
  uint number_of_leaf_intersections;
  uint num_active_leaves;

  real3 inv_bin_size;
  host_vector<uint> bins_intersected;
  host_vector<uint> bin_number;
  host_vector<uint> bin_number_out;
  host_vector<uint> bin_aabb_number;
  host_vector<uint> bin_start_index;
  host_vector<uint> num_contact;

  host_vector<uint> leaves_intersected;
  host_vector<uint> leaves_per_bin;
  host_vector<uint> leaf_number;
  host_vector<uint> leaf_number_out;
  host_vector<uint> leaf_aabb_number;
  host_vector<uint> leaf_start_index;

  //host_vector<short2> fam_data;
  //host_vector<bool> obj_active;
  //host_vector<uint> obj_data_id;

uint num_rigid_shapes;
uint num_rigid_bodies;


};
}
}

