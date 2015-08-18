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
// Authors: Radu Serban
// =============================================================================
//
// Miscelaneous common declarations and functions for the Chrono Parallel Utils
// library.
//
// =============================================================================

#ifndef CH_UTILS_COMMON_H
#define CH_UTILS_COMMON_H

#include <random>


namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// Construct a single random engine (on first use)
//
// Note that this object is never destructed (but this is OK)
// -----------------------------------------------------------------------------
inline std::default_random_engine& rengine() {
  static std::default_random_engine* re = new std::default_random_engine;
  return *re;
}

// -----------------------------------------------------------------------------
// sampleTruncatedDist
//
// Utility function for generating samples from a truncated normal distribution.
// -----------------------------------------------------------------------------
template <typename T>
inline T sampleTruncatedDist(std::normal_distribution<T>& distribution, T minVal, T maxVal) {
  T val;

  do {
    val = distribution(rengine());
  } while (val < minVal || val > maxVal);

  return val;
}


}  // end namespace utils
}  // end namespace chrono

#endif
