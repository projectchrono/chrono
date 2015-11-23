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
  ChParallelDataManager* data_manager;
 private:
  uint num_bins_active;
  uint number_of_bin_intersections;
  uint number_of_contacts_possible;

  custom_vector<uint> bins_intersected;
  custom_vector<uint> bin_number;
  custom_vector<uint> bin_number_out;
  custom_vector<uint> aabb_number;
  custom_vector<uint> bin_start_index;
  custom_vector<uint> num_contact;

};
}
}

