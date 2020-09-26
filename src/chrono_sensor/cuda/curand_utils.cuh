// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CURANDUTILS_H
#define CURANDUTILS_H

namespace chrono {
namespace sensor {

/// @addtogroup sensor_cuda
/// @{

/// Device function for initialing random values for cuRAND.
/// @param seed Random number generator seed.
/// @param rng_states The states to be randomly generated.
/// @param n_generators The number of random value generators we need.
__global__ void init_random_states(unsigned int seed, curandState_t* rng_states, int n_generators);

/// Host function for initialing random values for cuRAND.
/// @param seed Random number generator seed.
/// @param rng_states The states to be randomly generated.
/// @param n_generators The number of random value generators we need.
void init_cuda_rng(unsigned int seed, curandState_t* rng_states, int n_generators);

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
