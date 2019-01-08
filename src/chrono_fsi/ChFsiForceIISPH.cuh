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
    /// override the ForceSPH via IISPH method
    void ForceSPH(SphMarkerDataD* otherSphMarkersD, FsiBodiesDataD* otherFsiBodiesD, FsiMeshDataD* fsiMeshD) override;
    /// a class internal method to compute the pressure using a Poisson equation
    void calcPressureIISPH(
        thrust::device_vector<Real4> velMassRigid_fsiBodies_D,  ///< velocity of each rigid body in the system
        thrust::device_vector<Real3> accRigid_fsiBodies_D,      ///< acceleration of each rigid body in the system
        thrust::device_vector<Real3> pos_fsi_fea_D,  ///< position of fea nodes that are seen by the FSI system
        thrust::device_vector<Real3> vel_fsi_fea_D,  ///< velocity of fea nodes that are seen by the FSI system
        thrust::device_vector<Real3> acc_fsi_fea_D,  ///< acceleration of fea nodes that are seen by the FSI system
        thrust::device_vector<Real> sumWij_inv,      ///< Volume occupied by each SPH marker (Sum(W_{ij}))^-1
        thrust::device_vector<Real>& p_old,          ///< old pressure values in the iterative pressure update
        thrust::device_vector<Real3> Normals,        ///< Normal vector defined at each SPH marker
        thrust::device_vector<Real> G_i,  ///< G_i is a 3x3 correction matrix for each marker used in gradient operator.
        thrust::device_vector<Real>
            L_i,  ///< L_i is a 3x3 correction matrix for each marker used in laplacian operator.
        thrust::device_vector<Real>& Color);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
