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
// Author: Milad Rakhsha
// =============================================================================

#ifndef CH_FSI_FORCEIISPH_H_
#define CH_FSI_FORCEIISPH_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiForce.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief Child class of ChForceParallel that implements the I2SPH method.
class CH_FSI_API ChFsiForceIISPH : public ChFsiForce {
  public:
    ChFsiForceIISPH(
        ChBce* otherBceWorker,                   ///< Pointer to the ChBce object that handles BCE markers
        SphMarkerDataD* otherSortedSphMarkersD,  ///< Information of markers in the sorted array on device
        ProximityDataD*
            otherMarkersProximityD,  ///< Pointer to the object that holds the proximity of the markers on device
        FsiGeneralData* otherFsiGeneralData,  ///< Pointer to the sph general data
        SimParams* otherParamsH,              ///< Pointer to the simulation parameters on host
        NumberOfObjects* otherNumObjects      ///< Pointer to number of objects, fluid and boundary markers, etc.
    );

    ~ChFsiForceIISPH();
    void Finalize() override;

  private:
    void ForceSPH(SphMarkerDataD* otherSphMarkersD, FsiBodiesDataD* otherFsiBodiesD, FsiMeshDataD* fsiMeshD) override;
    void calcPressureIISPH(thrust::device_vector<Real4> velMassRigid_fsiBodies_D,
                           thrust::device_vector<Real3> accRigid_fsiBodies_D,
                           thrust::device_vector<Real3> pos_fsi_fea_D,
                           thrust::device_vector<Real3> vel_fsi_fea_D,
                           thrust::device_vector<Real3> acc_fsi_fea_D,
                           thrust::device_vector<Real> sumWij_inv,
                           thrust::device_vector<Real>& p_old,
                           thrust::device_vector<Real3> Normals,
                           thrust::device_vector<Real> G_i,
                           thrust::device_vector<Real> L_i,
                           thrust::device_vector<Real>& Color);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
