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
/// @param seed Random number generator seed. 64-bit, matching curand_init's own seed parameter,
/// because callers pack a per-stream identity into it; see ChSensorManager::GetDeterministicSeed.
/// @param rng_states The states to be randomly generated.
/// @param n_generators The number of random value generators we need.
__global__ void init_random_states(unsigned long long seed, curandState_t* rng_states, unsigned int n_generators);

/// Host function for initialing random values for cuRAND.
/// @param seed Random number generator seed. Obtain it from
/// ChSensorManager::GetDeterministicSeed(sensor, usage, stream_index) rather than from a global, so
/// that each RNG buffer gets a stream of its own.
/// @param rng_states The states to be randomly generated.
/// @param n_generators The number of random value generators we need.
void init_cuda_rng(unsigned long long seed, curandState_t* rng_states, unsigned int n_generators);

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
