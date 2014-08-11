// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Description: Base class for solver and constraint classes.
// This class provides indexing and common size variables
// =============================================================================

#ifndef CHBASEPARALLEL_H
#define CHBASEPARALLEL_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

namespace chrono {

class CH_PARALLEL_API ChBaseParallel {
 public:
   ChBaseParallel() {
   }
   ~ChBaseParallel() {
   }
   void Initialize();
 protected:
   ChParallelDataManager *data_container;
   uint num_bodies;
   uint num_contacts;
   uint num_unilaterals;
   uint num_bilaterals;
   uint num_constraints;

   real step_size;
   real alpha;
   real contact_recovery_speed;
};

}

#endif
