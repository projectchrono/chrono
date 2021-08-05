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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Utility function to print the save fluid, bce, and boundary data into file
// =============================================================================
#ifndef CHUTILSPRINTSPH_H
#define CHUTILSPRINTSPH_H
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/physics/ChParams.cuh"
#include "chrono_fsi/math/custom_math.h"
#define FLOAT16_TYPE_AVAILABLE
#include "chrono_thirdparty/chpf/particle_writer.hpp"

struct SimParams;

namespace chrono {
namespace fsi {
namespace utils {

/// function to save the fluid data into file
/// When called, this function creates three files to write fluid,
/// fluid-boundary and BCE markers data into file

CH_FSI_API void PrintToFile(const thrust::device_vector<Real4>& posRadD,
                            const thrust::device_vector<Real3>& velMasD,
                            const thrust::device_vector<Real4>& rhoPresMuD,
                            const thrust::device_vector<Real4>& sr_tau_I_mu_i,
                            const thrust::host_vector<int4>& referenceArray,
                            const thrust::host_vector<int4>& referenceArrayFEA,
                            const std::string& out_dir,
                            bool printToParaview = false);

/// helper function to save particle info from FSI system to a CSV file
/// this function saves particle positions, velocities, rho, pressure, and mu
CH_FSI_API void WriteCsvParticlesToFile(thrust::device_vector<Real4>& posRadD,
                                        thrust::device_vector<Real3>& velMasD,
                                        thrust::device_vector<Real4>& rhoPresMuD,
                                        thrust::host_vector<int4>& referenceArray,
                                        const std::string& outfilename);

/// helper function to save particle info from FSI system to a ChPF binary file
/// this function saves only particle positions.
CH_FSI_API void WriteChPFParticlesToFile(thrust::device_vector<Real4>& posRadD,
                                         thrust::host_vector<int4>& referenceArray,
                                         const std::string& outfilename);

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
