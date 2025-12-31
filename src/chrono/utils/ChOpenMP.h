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

#ifndef CHOPENMP_H
#define CHOPENMP_H

#include "chrono/core/ChApiCE.h"

namespace chrono {

/// Wrappers around OpenMP functions.
class ChApi ChOMP {
  public:
    /// Set the number of threads in subsequent parallel regions, unless overridden by a 'num_threads' clause
    static void SetNumThreads(int nthreads);

    /// Return the number of threads in the parallel region.
    static int GetNumThreads();

    /// Return the thread number of the thread executing within its thread team.
    static int GetThreadNum();

    /// Return the number of available processors on this machine
    static int GetNumProcs();

    /// Return the max. number of threads that would be used by default if num_threads not specified.
    /// This is the same number as GetNumProcs() on most OMP implementations.
    static int GetMaxThreads();
};

}  // end namespace chrono

#endif
