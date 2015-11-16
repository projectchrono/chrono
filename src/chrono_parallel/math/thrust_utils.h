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
// Description: simple linear algebra functions using thrust
// =============================================================================

#pragma once
#include "chrono_parallel/math/ChParallelMath.h"

// Thrust Includes
#include <thrust/host_vector.h>

#define Thrust_Inclusive_Scan_Sum(x, y)                    \
    thrust::inclusive_scan(x.begin(), x.end(), x.begin()); \
    y = x.back();
#define Thrust_Sort_By_Key(x, y) thrust::sort_by_key(x.begin(), x.end(), y.begin())
#define Thrust_Reduce_By_KeyA(x, y, z)                                                                                 \
    x = (thrust::reduce_by_key(y.begin(), y.end(), thrust::constant_iterator<uint>(1), y.begin(), z.begin()).second) - \
        z.begin()

#define Thrust_Reduce_By_Key(y, z, w)                                                                              \
    (thrust::reduce_by_key(y.begin(), y.end(), thrust::constant_iterator<uint>(1), z.begin(), w.begin()).second) - \
        w.begin()
#define Thrust_Inclusive_Scan(x) thrust::inclusive_scan(x.begin(), x.end(), x.begin())
#define Thrust_Exclusive_Scan(x) thrust::exclusive_scan(x.begin(), x.end(), x.begin())
#define Thrust_Fill(x, y) thrust::fill(x.begin(), x.end(), y)
#define Thrust_Sort(x) thrust::sort(x.begin(), x.end())
#define Thrust_Count(x, y) thrust::count(x.begin(), x.end(), y)
#define Thrust_Sequence(x) thrust::sequence(x.begin(), x.end())
#define Thrust_Equal(x, y) thrust::equal(x.begin(), x.end(), y.begin())
#define Thrust_Max(x) x[thrust::max_element(x.begin(), x.end()) - x.begin()]
#define Thrust_Min(x) x[thrust::min_element(x.begin(), x.end()) - x.begin()]
#define Thrust_Total(x) thrust::reduce(x.begin(), x.end())
#define Thrust_Unique(x) thrust::unique(x.begin(), x.end()) - x.begin();


template <class T>
static inline std::ostream& operator<<(std::ostream& out, const std::vector<T>& x) {
  for (uint i = 0; i < x.size(); i++) {
    out << x[i] << std::endl;
  }
  return out;
}

