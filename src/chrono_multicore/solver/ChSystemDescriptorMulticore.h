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

#include "chrono/solver/ChSystemDescriptor.h"

#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"
#include "chrono/multicore_math/ChMulticoreMath.h"

namespace chrono {

/// @addtogroup multicore_solver
/// @{

/// System descriptor for Chrono::Multicore.
class CH_MULTICORE_API ChSystemDescriptorMulticore : public ChSystemDescriptor {
  public:
    ChSystemDescriptorMulticore(ChMulticoreDataManager* dc) {}
    ~ChSystemDescriptorMulticore() {}
};

/// @} multicore_solver

}  // end namespace chrono
