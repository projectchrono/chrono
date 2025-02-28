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

#include "chrono_fsi/sph/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

struct SimParams;

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsi_utils
/// @{

/// Save current CFD SPH data to files.
/// Create separate files to write fluid, boundary BCE, rigid BCE, and flex BCE marker information.
/// The amount of data saved for each marker is controlled by the specified OutputLevel (e.g., for level CFD_FULL, the
/// output includes shear rate).
void SaveParticleDataCFD(const std::string& dir, OutputLevel level, FsiDataManager& data_mgr);
void SaveParticleDataCFD(const std::string& dir,
                         OutputLevel level,
                         const thrust::device_vector<Real4>& posRadD,
                         const thrust::device_vector<Real3>& velMasD,
                         const thrust::device_vector<Real4>& derivVelRhoD,
                         const thrust::device_vector<Real4>& rhoPresMuD,
                         const thrust::device_vector<Real4>& srTauMuD,
                         const thrust::host_vector<int4>& referenceArray,
                         const thrust::host_vector<int4>& referenceArrayFEA);

/// Save current CRM SPH data to files.
/// Create separate files to write fluid, boundary BCE, rigid BCE, and flex BCE marker information.
/// The amount of data saved for each marker is controlled by the specified OutputLevel (e.g., for level CRM_FULL, the
/// output includes stress).
void SaveParticleDataCRM(const std::string& dir, OutputLevel level, FsiDataManager& data_mgr);
void SaveParticleDataCRM(const std::string& dir,
                         OutputLevel level,
                         const thrust::device_vector<Real4>& posRadD,
                         const thrust::device_vector<Real3>& velMasD,
                         const thrust::device_vector<Real4>& derivVelRhoD,
                         const thrust::device_vector<Real4>& rhoPresMuD,
                         const thrust::device_vector<Real3>& tauXxYyZzD,
                         const thrust::device_vector<Real3>& tauXyXzYzD,
                         const thrust::host_vector<int4>& referenceArray,
                         const thrust::host_vector<int4>& referenceArrayFEA);

/// Save current FSI solid data.
/// Append states and fluid forces at current time for all solids in the FSI problem.
void SaveSolidData(const std::string& dir, double time, FsiDataManager& data_mgr);
void SaveSolidData(const std::string& dir,
                   double time,
                   const thrust::device_vector<Real3>& posRigidD,
                   const thrust::device_vector<Real4>& rotRigidD,
                   const thrust::device_vector<Real3>& velRigidD,
                   const thrust::device_vector<Real3>& forceRigidD,
                   const thrust::device_vector<Real3>& torqueRigidD,
                   const thrust::device_vector<Real3>& pos1DNodeD,
                   const thrust::device_vector<Real3>& vel1DNodeD,
                   const thrust::device_vector<Real3>& force1DNodeD,
                   const thrust::device_vector<Real3>& pos2DNodeD,
                   const thrust::device_vector<Real3>& vel2DNodeD,
                   const thrust::device_vector<Real3>& force2DNodeD);

/// Save current particle data to a CSV file.
/// Write particle positions, velocities, rho, pressure, and mu.
void WriteParticleFileCSV(const std::string& filename, FsiDataManager& data_mgr);
void WriteParticleFileCSV(const std::string& filename,
                          thrust::device_vector<Real4>& posRadD,
                          thrust::device_vector<Real3>& velMasD,
                          thrust::device_vector<Real4>& rhoPresMuD,
                          thrust::host_vector<int4>& referenceArray);

/// @} fsi_utils

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
