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
// This class generates an AABB for every collision shape
// =============================================================================

#ifndef CHC_AABBGENERATOR_H
#define CHC_AABBGENERATOR_H

#include "collision/ChCCollisionModel.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {
namespace collision {

class CH_PARALLEL_API ChCAABBGenerator {
 public:
  // functions
  ChCAABBGenerator();

  void GenerateAABB();

  ChParallelDataManager* data_manager;
};
}
}

#endif
