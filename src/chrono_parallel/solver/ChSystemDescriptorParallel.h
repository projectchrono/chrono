// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#pragma once

//#include <thrust/count.h>

#include "chrono/solver/ChSystemDescriptor.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

/// @addtogroup parallel_solver
/// @{

/// System descriptor for Chrono::Parallel.
class CH_PARALLEL_API ChSystemDescriptorParallel : public ChSystemDescriptor {
  public:
    ChSystemDescriptorParallel(ChParallelDataManager* dc) : data_manager(dc) {}
    ~ChSystemDescriptorParallel() {}

  private:
    ChParallelDataManager* data_manager;  ///< Pointer to the system's data manager
};

/// @} parallel_solver

} // end namespace chrono
