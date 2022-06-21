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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
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
#include "chrono_fsi/physics/ChParams.h"
#include "chrono_fsi/math/custom_math.h"
#define FLOAT16_TYPE_AVAILABLE
#include "chrono_thirdparty/chpf/particle_writer.hpp"

struct SimParams;

namespace chrono {
namespace fsi {
namespace utils {

/// @addtogroup fsi_utils
/// @{

/// Helper function to save the SPH data into files.
/// When called, this function creates three files to write fluid,
/// boundary and BCE particles data into files.
CH_FSI_API void PrintToFile(const thrust::device_vector<Real4>& posRadD,
                            const thrust::device_vector<Real3>& velMasD,
                            const thrust::device_vector<Real4>& rhoPresMuD,
                            const thrust::device_vector<Real4>& sr_tau_I_mu_i,
                            const thrust::host_vector<int4>& referenceArray,
                            const thrust::host_vector<int4>& referenceArrayFEA,
                            const std::string& dir,
                            const std::shared_ptr<SimParams>& paramsH,
                            bool printToParaview = false);

/// Helper function to save particle info from FSI system to a CSV files. 
/// This function saves particle positions, velocities, rho, pressure, and mu.
CH_FSI_API void WriteCsvParticlesToFile(thrust::device_vector<Real4>& posRadD,
                                        thrust::device_vector<Real3>& velMasD,
                                        thrust::device_vector<Real4>& rhoPresMuD,
                                        thrust::host_vector<int4>& referenceArray,
                                        const std::string& outfilename);

/// Helper function to save particle info from FSI system to a ChPF binary files.
/// This function saves only particle positions.
CH_FSI_API void WriteChPFParticlesToFile(thrust::device_vector<Real4>& posRadD,
                                         thrust::host_vector<int4>& referenceArray,
                                         const std::string& outfilename);

/// @} fsi_utils

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
