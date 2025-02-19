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

#include "chrono/utils/ChOpenMP.h"

#ifdef _OPENMP
    #include <omp.h>
#endif

namespace chrono {

#ifdef _OPENMP

inline void ChOMP::SetNumThreads(int nthreads) {
    omp_set_num_threads(nthreads);
}

inline int ChOMP::GetNumThreads() {
    return omp_get_num_threads();
}

inline int ChOMP::GetThreadNum() {
    return omp_get_thread_num();
}

inline int ChOMP::GetNumProcs() {
    return omp_get_num_procs();
}

inline int ChOMP::GetMaxThreads() {
    return omp_get_max_threads();
}

#else

inline void ChOMP::SetNumThreads(int nthreads) {}

inline int ChOMP::GetNumThreads() {
    return 1;
}

inline int ChOMP::GetThreadNum() {
    return 1;
}

inline int ChOMP::GetNumProcs() {
    return 1;
}

inline int ChOMP::GetMaxThreads() {
    return 1;
}

#endif

}  // end namespace chrono
