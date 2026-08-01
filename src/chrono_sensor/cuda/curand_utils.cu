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

#include "chrono_sensor/cuda/curand_utils.cuh"

namespace chrono {
namespace sensor {

// The seed is 64-bit because curand_init's seed parameter is, and because callers pack a stream
// identity (manager, sensor, filter, purpose) into it. Narrowing to 32 bits here would truncate
// the high fields of that identity and silently re-merge streams that the caller separated.
__global__ void init_random_states(unsigned long long seed, curandState_t* rng_states, unsigned int n_generators) {
    // unsigned throughout: the count cannot be negative, and mixing an int index with an unsigned
    // bound would compare them after converting the index to unsigned.
    unsigned int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index < n_generators)
        curand_init(seed, index, 0, &rng_states[index]);
}

void init_cuda_rng(unsigned long long seed, curandState_t* rng_states, unsigned int n_generators) {
    const unsigned int nThreads = 512;
    unsigned int nBlocks = (n_generators + nThreads - 1) / nThreads;
    init_random_states<<<nBlocks, nThreads>>>(seed, rng_states, n_generators);
}

}  // namespace sensor
}  // namespace chrono
