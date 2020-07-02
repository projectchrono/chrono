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

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

namespace chrono {
namespace sensor {

__global__ void init_random_states(unsigned int seed, curandState_t* rng_states, int n_generators) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index < n_generators) {
        curand_init(seed, index, 0, &rng_states[index]);
    }
}
}  // namespace sensor
}  // namespace chrono
